//! Wraps the lsm9ds1 library to provide an interface more closely matching that of the Buckler.
//! https://github.com/lab11/buckler/blob/master/software/libraries/lsm9ds1/lsm9ds1.h
use lsm9ds1 as imu_mod;
use lsm9ds1::LSM9DS1Init;
use nrf52832_hal::{timer, twim, Twim};

pub struct Imu<S: twim::Instance, T: timer::Instance> {
    instance: imu_mod::LSM9DS1<imu_mod::interface::I2cInterface<Twim<S>>>,
    timer: T,
    integration_started: bool,
    integrated_angle: ImuMeasure,
    prev_timer_val: u32,
}

#[derive(Debug, Copy, Clone)]
pub struct ImuMeasure {
    pub x_axis: f32,
    pub y_axis: f32,
    pub z_axis: f32,
}

impl ImuMeasure {
    fn from_triple(triple: (f32, f32, f32)) -> Self {
        ImuMeasure {
            x_axis: triple.0,
            y_axis: triple.1,
            z_axis: triple.2,
        }
    }
}

impl<S: twim::Instance, T: timer::Instance> Imu<S, T> {
    pub fn new(twi0: Twim<S>, timer: T) -> Self {
        let mut instance = IMU_CONF.with_interface(imu_mod::interface::I2cInterface::init(
            twi0,
            imu_mod::interface::i2c::AgAddress::_1,  // 0x6A
            imu_mod::interface::i2c::MagAddress::_1, // 0x1C
        ));
        instance.begin_accel().unwrap();
        instance.begin_gyro().unwrap();
        instance.begin_mag().unwrap();
        Imu {
            instance,
            timer,
            integration_started: false,
            integrated_angle: ImuMeasure::from_triple((0.0, 0.0, 0.0)),
            prev_timer_val: 0,
        }
    }

    /// Read all three axes on the accelerometer
    ///
    /// Return measurements as floating point values in g's
    pub fn read_accel(
        &mut self,
    ) -> Result<ImuMeasure, imu_mod::interface::i2c::Error<twim::Error>> {
        Ok(ImuMeasure::from_triple(self.instance.read_accel()?))
    }

    /// Read all three axes on the gyro
    ///
    /// Return measurements as floating point values in degrees/second
    pub fn read_gyro(&mut self) -> Result<ImuMeasure, imu_mod::interface::i2c::Error<twim::Error>> {
        Ok(ImuMeasure::from_triple(self.instance.read_gyro()?))
    }

    /// Read all three axes on the magnetometer
    ///
    /// Return measurements as floating point values in uT
    pub fn read_mag(&mut self) -> Result<ImuMeasure, imu_mod::interface::i2c::Error<twim::Error>> {
        Ok(ImuMeasure::from_triple(self.instance.read_mag()?))
    }

    /// Forcibly restarts gyro integration, regardless of whether it was already started.
    /// This is akin to calling stop followed by start.
    pub fn restart_gyro_integration(&mut self) {
        self.stop_gyro_integration();
        self.start_gyro_integration();
    }

    // TODO add type states for integration
    /// Begins integration on the gyro. Panics if integration already started.
    pub fn start_gyro_integration(&mut self) {
        assert!(!self.integration_started);
        self.integration_started = true;
        self.integrated_angle = ImuMeasure::from_triple((0.0, 0.0, 0.0));
        self.timer.disable_interrupt();
        self.timer.set_oneshot();
        self.timer.timer_start(u32::MAX);
        self.prev_timer_val = 0;
    }

    /// Read the value of the integrated gyro
    ///
    /// Note: this function also performs the integration and needs to be called
    /// periodically
    ///
    /// Return the integrated value as floating point in degrees
    pub fn read_gyro_integration(
        &mut self,
    ) -> Result<ImuMeasure, imu_mod::interface::i2c::Error<twim::Error>> {
        let curr_timer_val = self.timer.read_counter();
        let time_diff = (curr_timer_val.wrapping_sub(self.prev_timer_val) as f32) / 1000000.0;
        self.prev_timer_val = curr_timer_val;
        let measure = self.read_gyro()?;
        if measure.z_axis > 0.5 || measure.z_axis < -0.5 {
            self.integrated_angle.z_axis += measure.z_axis * time_diff;
        }
        if measure.x_axis > 0.5 || measure.x_axis < -0.5 {
            self.integrated_angle.x_axis += measure.x_axis * time_diff;
        }
        if measure.y_axis > 0.5 || measure.y_axis < -0.5 {
            self.integrated_angle.y_axis += measure.y_axis * time_diff;
        }
        Ok(self.integrated_angle)
    }

    /// Stops integration on the gyro.
    pub fn stop_gyro_integration(&mut self) {
        self.timer.timer_cancel();
        self.integration_started = false;
    }
}

const IMU_CONF: LSM9DS1Init = LSM9DS1Init {
    accel: imu_mod::accel::AccelSettings {
        enable_x: true,
        enable_y: true,
        enable_z: true,
        sample_rate: imu_mod::accel::ODR::_952Hz,
        scale: imu_mod::accel::Scale::_2G,
        bandwidth_selection: imu_mod::accel::BandwidthSelection::ByODR,
        bandwidth: imu_mod::accel::Bandwidth::_408Hz, // This shouldn't matter since we determine by ODR
        high_res_bandwidth: imu_mod::accel::HighRes::Disabled,
    },
    gyro: imu_mod::gyro::GyroSettings {
        enable_x: true,
        enable_y: true,
        enable_z: true,
        flip_x: false,
        flip_y: false,
        flip_z: false,
        scale: imu_mod::gyro::Scale::_245DPS,
        sample_rate: imu_mod::gyro::ODR::_952Hz,
        bandwidth: imu_mod::gyro::Bandwidth::LPF_0,
        int_selection: imu_mod::gyro::GyroIntSelection::SEL_0,
        out_selection: imu_mod::gyro::GyroOutSelection::SEL_0,
        low_power_mode: imu_mod::gyro::LowPowerMode::Disabled,
        hpf_mode: imu_mod::gyro::HpFilter::Disabled,
        hpf_cutoff: imu_mod::gyro::HpFilterCutoff::HPCF_1,
        latch_interrupt: imu_mod::gyro::LatchInterrupt::Enabled,
    },
    mag: imu_mod::mag::MagSettings {
        sample_rate: imu_mod::mag::ODR::_80Hz,
        temp_compensation: imu_mod::mag::TempComp::Disabled,
        x_y_performance: imu_mod::mag::OpModeXY::UltraHigh,
        scale: imu_mod::mag::Scale::_4G,
        i2c_mode: imu_mod::mag::I2cMode::Enabled,
        system_op: imu_mod::mag::SysOpMode::Continuous,
        low_power: imu_mod::mag::LowPowerMode::Disabled,
        spi_mode: imu_mod::mag::SpiMode::W, // Doesn't matter since we use I2C
        z_performance: imu_mod::mag::OpModeZ::UltraHigh,
    },
};
