//! Wraps the lsm9ds1 library to provide an interface more closely matching that of the Buckler.
//! https://github.com/lab11/buckler/blob/master/software/libraries/lsm9ds1/lsm9ds1.h
use lsm9ds1 as imu_mod;
use lsm9ds1::LSM9DS1Init;
use nrf52832_hal::{twim, Twim};

pub struct Imu<I: twim::Instance> {
    instance: imu_mod::LSM9DS1<imu_mod::interface::I2cInterface<Twim<I>>>,
}

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

impl<I: twim::Instance> Imu<I> {
    pub fn new(twi0: Twim<I>) -> Self {
        let mut instance = IMU_CONF.with_interface(imu_mod::interface::I2cInterface::init(
            twi0,
            imu_mod::interface::i2c::AgAddress::_1,  // 0x6A
            imu_mod::interface::i2c::MagAddress::_1, // 0x1C
        ));
        instance.begin_accel().unwrap();
        instance.begin_gyro().unwrap();
        instance.begin_mag().unwrap();
        Imu { instance }
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

    // TODO add type states for integration
    /// Beings integration on the gyro. Panics if integration already started.
    pub fn start_gyro_integration(&mut self) {
        unimplemented!()
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
