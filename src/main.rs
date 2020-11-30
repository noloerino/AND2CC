#![no_main]
#![no_std]

mod ble_channel;
mod ble_service;
mod buckler;
mod error;
mod examples;
mod kobuki;
mod pixy2;

use core::default;
use core::fmt::Write;
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52832_hal as hal;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use rubble::l2cap::{BleChannelMap, L2CAPState};
use rubble::link::queue::{PacketQueue, SimpleQueue};
use rubble::link::{
    ad_structure::AdStructure, AddressKind, DeviceAddress, LinkLayer, Responder, MIN_PDU_BUF,
};
use rubble::time::{Duration, Timer};
use rubble::{config::Config, security::NoSecurity};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;

const DETECT_RECALIBRATE_M: f32 = 1.0;
const DRIVE_SPEED: i16 = 70;

/// Top-level states of the FSM
#[derive(PartialEq)]
enum TopState {
    Off,
    Detect(DetectState),
    Dock,
    Drive,
}

impl default::Default for TopState {
    fn default() -> Self {
        TopState::Off
    }
}

/// Describes states for the detection phase.
#[derive(PartialEq)]
enum DetectState {
    /// Rotating to look for target
    Scan,
    /// Driving towards the target (should be facing backwards)
    Approach {
        last_encoder: u16,
        distance_traveled: f32,
    },
}

#[derive(PartialEq)]
enum DriveDirection {
    Forward,
    Reverse,
}

fn measure_distance(curr_encoder: u16, prev_encoder: u16, direction: DriveDirection) -> f32 {
    const CONVERSION: f32 = 0.0006108;
    // rprintln!("encoder: {} -> {}", prev_encoder, curr_encoder);
    CONVERSION
        * (if direction == DriveDirection::Forward {
            (if curr_encoder >= prev_encoder {
                curr_encoder - prev_encoder
            } else {
                curr_encoder + (u16::MAX - prev_encoder)
            }) as f32
        } else {
            -((if curr_encoder <= prev_encoder {
                prev_encoder - curr_encoder
            } else {
                prev_encoder + (u16::MAX - curr_encoder)
            }) as f32)
        })
}

pub enum BleConfig {}

impl Config for BleConfig {
    type Timer = BleTimer<hal::pac::TIMER0>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<ble_service::RomiServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

#[app(device = nrf52832_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        b: buckler::board::Board,
        // Below resources are needed for BLE - see demo
        // https://github.com/jonas-schievink/rubble/blob/master/demos/nrf52-demo/src/main.rs
        #[init([0; MIN_PDU_BUF])]
        ble_tx_buf: PacketBuffer,
        #[init([0; MIN_PDU_BUF])]
        ble_rx_buf: PacketBuffer,
        #[init(SimpleQueue::new())]
        tx_queue: SimpleQueue,
        #[init(SimpleQueue::new())]
        rx_queue: SimpleQueue,
        ble_ll: LinkLayer<BleConfig>,
        ble_r: Responder<BleConfig>,
        radio: BleRadio,
    }

    // https://rtic.rs/0.5/book/en/by-example/app.html#init
    #[init(resources = [ble_tx_buf, ble_rx_buf, tx_queue, rx_queue])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        let p: hal::pac::Peripherals = cx.device;
        let c: hal::pac::CorePeripherals = cx.core;
        // Enable pin reset before anything else
        if p.UICR.pselreset[0].read().bits() != 21 || p.UICR.pselreset[1].read().bits() != 21 {
            unsafe {
                p.UICR.pselreset[0].write(|w| w.pin().bits(21));
                p.UICR.pselreset[1].write(|w| w.pin().bits(21));
            }
            hal::pac::SCB::sys_reset();
        }
        // Set up board components
        let b = buckler::board::Board::new(
            buckler::board::BoardInitResources {
                P0: p.P0,
                UARTE0: p.UARTE0,
                SPIM1: p.SPIM1,
                SPIM2: p.SPIM2,
                TWIM0: p.TWIM0,
                TIMER1: p.TIMER1,
                TIMER2: p.TIMER2,
            },
            c,
        );
        // Set up BLE clocks etc.
        // Everything here is copied from the demo w/ slight modification, minus serial and logger
        let _clocks = hal::clocks::Clocks::new(p.CLOCK).enable_ext_hfosc();
        let ble_timer = BleTimer::init(p.TIMER0);
        // Device address is transmitted by LSB first
        // c0:98:e5:49:xx:xx is specified by the lab
        // If NRF connect is displaying something weird, it's probably some caching issue
        // which can be circumvented by changing the LSB
        let device_address =
            DeviceAddress::new([0x03, 0x00, 0x49, 0xE5, 0x98, 0xC0], AddressKind::Public);
        let mut radio = BleRadio::new(
            p.RADIO,
            &p.FICR,
            cx.resources.ble_tx_buf,
            cx.resources.ble_rx_buf,
        );
        // Create TX/RX queues
        let (tx, tx_cons) = cx.resources.tx_queue.split();
        let (rx_prod, rx) = cx.resources.rx_queue.split();
        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::<BleConfig>::new(device_address, ble_timer);
        let ble_r = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(
                ble_service::RomiServiceAttrs::new(),
            )),
        );
        // Send advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                Duration::from_millis(1000),
                &[AdStructure::CompleteLocalName("EE149 | DDD")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();
        ble_ll.timer().configure_interrupt(next_update);
        init::LateResources {
            b,
            radio,
            ble_ll,
            ble_r,
        }
    }

    #[task(binds = RADIO, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn radio(ctx: radio::Context) {
        let ble_ll: &mut LinkLayer<BleConfig> = ctx.resources.ble_ll;
        if let Some(cmd) = ctx
            .resources
            .radio
            .recv_interrupt(ble_ll.timer().now(), ble_ll)
        {
            ctx.resources.radio.configure_receiver(cmd.radio);
            ble_ll.timer().configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                ctx.spawn.ble_worker().ok();
            }
        }
    }

    #[task(binds = TIMER0, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        let timer = ctx.resources.ble_ll.timer();
        if !timer.is_interrupt_pending() {
            return;
        }
        timer.clear_interrupt();

        let cmd = ctx.resources.ble_ll.update_timer(ctx.resources.radio);
        ctx.resources.radio.configure_receiver(cmd.radio);

        ctx.resources
            .ble_ll
            .timer()
            .configure_interrupt(cmd.next_update);

        if cmd.queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            ctx.spawn.ble_worker().ok();
        }
    }

    #[task(resources = [ble_r], priority = 2)]
    fn ble_worker(ctx: ble_worker::Context) {
        // Fully drain the packet queue
        while ctx.resources.ble_r.has_work() {
            ctx.resources.ble_r.process_one().unwrap();
        }
    }

    extern "C" {
        fn WDT();
    }

    #[idle(resources = [b])]
    fn idle(c: idle::Context) -> ! {
        let b = c.resources.b;
        // main_loop(b);
        // Comment out main_loop and uncomment these to run sanity examples
        examples::blink(b);
        // examples::display(b);
        // examples::pixy(b);
        // examples::drive_forward(b);
        // examples::drive_reverse(b);
        // examples::dock_continuity(b);
        // examples::target_block(b);
    }
};

fn main_loop(b: &mut buckler::board::Board) -> ! {
    use TopState::*;
    let mut top_state = TopState::default();
    loop {
        b.delay.delay_ms(1u8);
        b.poll_sensors().unwrap();
        // Can't just print debug string due to internal state
        b.display
            .row_0()
            .write_str(match top_state {
                Off => "Off",
                Detect(..) => "Detect",
                Dock => "Dock",
                Drive => "Drive",
            })
            .ok();
        let is_button_pressed = b.sensors.is_button_pressed();
        match top_state {
            Off => {
                b.display.row_1().write_str("").ok();
                if is_button_pressed {
                    rprintln!("Beginning detect phase");
                    b.imu.restart_gyro_integration();
                    top_state = TopState::Detect(DetectState::Scan);
                } else {
                    b.actuator().drive_direct(0, 0).ok();
                }
            }
            Detect(detect_state) => {
                // TODO transition to dock when proximity is detected
                if is_button_pressed {
                    top_state = TopState::Off;
                } else {
                    top_state = Detect(detect_state.react(b));
                }
            }
            _ => unimplemented!(),
        }
    }
}

impl DetectState {
    fn react(self, b: &mut buckler::board::Board) -> DetectState {
        use DetectState::*;
        // TODO hook up to pixy2
        // Hack to simulate detection after some number of cycles
        static mut N: u32 = 0;
        let tgt_detected: bool;
        unsafe {
            match self {
                Scan => {
                    N += 1;
                }
                _ => N = 0,
            }
            tgt_detected = N >= 200;
        }
        match self {
            Scan => {
                let angle = fabs(b.imu.read_gyro_integration().unwrap().z_axis);
                if tgt_detected {
                    rprintln!("Moving to approach at angle {}", angle);
                    b.imu.stop_gyro_integration();
                    Approach {
                        last_encoder: b.sensors.left_wheel_encoder,
                        distance_traveled: 0.0,
                    }
                } else {
                    // If this turns out to be flaky, ok() instead of unwrap() and retry
                    b.display
                        .row_1()
                        .write_fmt(format_args!("SCAN: {:.1}", angle))
                        .ok();
                    b.actuator().drive_direct(DRIVE_SPEED, -DRIVE_SPEED).ok();
                    Scan
                }
            }
            Approach {
                last_encoder,
                mut distance_traveled,
            } => {
                b.display
                    .row_1()
                    .write_fmt(format_args!("APPROACH: {:.1}m", distance_traveled))
                    .ok();
                if distance_traveled >= DETECT_RECALIBRATE_M {
                    rprintln!("Reorienting towards target");
                    b.imu.start_gyro_integration();
                    Scan
                } else {
                    // Drive robot backwards until 1m has been traversed, at which point we attempt
                    // to reorient just to be safe
                    b.actuator().drive_direct(-DRIVE_SPEED, -DRIVE_SPEED).ok();
                    let curr_encoder = b.sensors.left_wheel_encoder;
                    distance_traveled += fabs(measure_distance(
                        curr_encoder,
                        last_encoder,
                        DriveDirection::Reverse,
                    ));
                    Approach {
                        last_encoder: curr_encoder,
                        distance_traveled,
                    }
                }
            }
        }
    }
}

/// Apparently f32::abs is part of std, not core.
fn fabs(n: f32) -> f32 {
    if n >= 0.0 {
        n
    } else {
        -n
    }
}
