use crate::errors::{DroneError, Result};
use embassy_rp::i2c::{Async, I2c, Instance, Mode};
use embassy_time::{Duration, Timer};
use modular_bitfield::bitfield;
use modular_bitfield::prelude::*;

pub const ADDR: u8 = 0xD0;
pub const DEFAULT_SLAVE_ADDR: u8 = 0x68;

macro_rules! mpu6050_regs {
    ($($name:ident : $val:expr), * $(,)?) => {
        $(
            pub const $name: u8 = $val;
        )*
    };
}

mpu6050_regs! {
    SELF_TEST_X: 0x0D,
    SELF_TEST_Y: 0x0E,
    SELF_TEST_Z: 0x0F,
    SELF_TEST_A: 0x10,
    SMPLRT_DIV: 0x19,
    CONFIG: 0x1A,
    GYRO_CONFIG: 0x1B,
    ACCEL_CONFIG: 0x1C,
    FIFO_EN: 0x23,
    I2C_MST_CTRL: 0x24,
    I2C_SLV0_ADDR: 0x25,
    I2C_SLV0_REG: 0x26,
    I2C_SLV0_CTRL: 0x27,
    I2C_SLV1_ADDR: 0x28,
    I2C_SLV1_REG: 0x29,
    I2C_SLV1_CTRL: 0x2A,
    I2C_SLV2_ADDR: 0x2B,
    I2C_SLV2_REG: 0x2C,
    I2C_SLV2_CTRL: 0x2D,
    I2C_SLV3_ADDR: 0x2E,
    I2C_SLV3_REG: 0x2F,
    I2C_SLV3_CTRL: 0x30,
    I2C_SLV4_ADDR: 0x31,
    I2C_SLV4_REG: 0x32,
    I2C_SLV4_DO: 0x33,
    I2C_SLV4_CTRL: 0x34,
    I2C_SLV4_DI: 0x35,
    I2C_MST_STATUS: 0x36,
    INT_PIN_CFG: 0x37,
    INT_ENABLE: 0x38,
    INT_STATUS: 0x3A,
    ACCEL_XOUT_H: 0x3B,
    ACCEL_XOUT_L: 0x3C,
    ACCEL_YOUT_H: 0x3D,
    ACCEL_YOUT_L: 0x3E,
    ACCEL_ZOUT_H: 0x3F,
    ACCEL_ZOUT_L: 0x40,
    TEMP_OUT_H: 0x41,
    TEMP_OUT_L: 0x42,
    GYRO_XOUT_H: 0x43,
    GYRO_XOUT_L: 0x44,
    GYRO_YOUT_H: 0x45,
    GYRO_YOUT_L: 0x46,
    GYRO_ZOUT_H: 0x47,
    GYRO_ZOUT_L: 0x48,
    EXT_SENS_DATA_00: 0x49,
    EXT_SENS_DATA_01: 0x4A,
    EXT_SENS_DATA_02: 0x4B,
    EXT_SENS_DATA_03: 0x4C,
    EXT_SENS_DATA_04: 0x4D,
    EXT_SENS_DATA_05: 0x4E,
    EXT_SENS_DATA_06: 0x4F,
    EXT_SENS_DATA_07: 0x50,
    EXT_SENS_DATA_08: 0x51,
    EXT_SENS_DATA_09: 0x52,
    EXT_SENS_DATA_10: 0x53,
    EXT_SENS_DATA_11: 0x54,
    EXT_SENS_DATA_12: 0x55,
    EXT_SENS_DATA_13: 0x56,
    EXT_SENS_DATA_14: 0x57,
    EXT_SENS_DATA_15: 0x58,
    EXT_SENS_DATA_16: 0x59,
    EXT_SENS_DATA_17: 0x5A,
    EXT_SENS_DATA_18: 0x5B,
    EXT_SENS_DATA_19: 0x5C,
    EXT_SENS_DATA_20: 0x5D,
    EXT_SENS_DATA_21: 0x5E,
    EXT_SENS_DATA_22: 0x5F,
    EXT_SENS_DATA_23: 0x60,
    I2C_SLV0_DO: 0x63,
    I2C_SLV1_DO: 0x64,
    I2C_SLV2_DO: 0x65,
    I2C_SLV3_DO: 0x66,
    I2C_MST_DELAY_CTRL: 0x67,
    SIGNAL_PATH_RESET: 0x68,
    USER_CTRL: 0x6A,
    PWR_MGMT_1: 0x6B,
    PWR_MGMT_2: 0x6C,
    FIFO_COUNTH: 0x72,
    FIFO_COUNTL: 0x73,
    FIFO_R_W: 0x74,
    WHO_AM_I: 0x75,
}

#[bitfield]
pub struct PwrMgmt1 {
    #[bits = 3]
    clksel: ClockSource,
    temp_dis: bool,
    #[skip] __: B1,
    cycle: bool,
    sleep: bool,
    device_reset: bool
}

#[derive(BitfieldSpecifier)]
pub enum ClockSource {
    Internal8MHz = 0,
    PllXAxis = 1,
    PllYAxis = 2,
    PllZAxis = 3,
    PllExternal32768kHz = 4,
    PllExternal192kHz = 5,
    Reserved = 6,
    StopClockReset = 7
}

#[bitfield]
pub struct AccelConfig {
    #[skip] __: B3,
    #[bits = 2]
    afs_sel: AccelRange,
    za_st: bool,
    ya_st: bool,
    xa_st: bool
}

#[derive(Debug, Eq, PartialEq, Copy, Clone, BitfieldSpecifier)]
pub enum AccelRange {
    G2 = 0,
    G4,
    G8,
    G16
}

#[bitfield]
pub struct GyroConfig {
    #[skip] __: B3,
    #[bits = 2]
    fs_sel: GyroRange,
    zg_st: bool,
    yg_st: bool,
    xg_st: bool
}

#[derive(BitfieldSpecifier)]
pub enum GyroRange {
    D250 = 0,
    D500,
    D1000,
    D2000
}

pub struct Mpu6050<'d, T: Instance, M: Mode> {
    i2c: I2c<'d, T, M>
}

impl<'d, T: Instance> Mpu6050<'d, T, Async> {
    pub fn new_async(i2c: I2c<'d, T, Async>) -> Self {
        Mpu6050 {
            i2c
        }
    }

    pub async fn id(&mut self) -> Result<u8> {
        self.read(WHO_AM_I).await
    }

    pub async fn wake(&mut self, delay: Duration) -> Result<()> {
        self.write(PWR_MGMT_1, &[0x0]).await?;
        Timer::after(delay).await;
        Ok(())
    }

    pub async fn init_with_gyro_accel_range(
        &mut self,
        delay: Duration,
        accel_range: AccelRange,
        gyro_range: GyroRange
    ) -> Result<()> {
        self.wake(delay).await?;
        self.verify().await?;
        self.set_accel_range(accel_range).await?;
        self.set_gyro_range(gyro_range).await?;
        Ok(())
    }

    #[inline]
    pub async fn init(&mut self, delay: Duration) -> Result<()> {
        self.init_with_gyro_accel_range(delay, AccelRange::G2, GyroRange::D250).await?;
        Ok(())
    }

    pub async fn verify(&mut self) -> Result<()> {
        let id = self.id().await?;
        if id != DEFAULT_SLAVE_ADDR {
            return Err(DroneError::InvalidChipId(id))
        }
        Ok(())
    }

    pub async fn set_accel_range(&mut self, accel_range: AccelRange) -> Result<()> {
        let accel_config = AccelConfig::new().with_afs_sel(accel_range);
        self.write(ACCEL_CONFIG, &accel_config.bytes).await?;
        Ok(())
    }

    pub async fn set_accel_range_with_self_test(&mut self, accel_range: AccelRange) -> Result<()> {
        let accel_config = AccelConfig::new()
            .with_afs_sel(accel_range)
            .with_xa_st(true)
            .with_ya_st(true)
            .with_za_st(true);
        self.write(ACCEL_CONFIG, &accel_config.bytes).await?;
        Ok(())
    }

    pub async fn set_gyro_range(&mut self, gyro_range: GyroRange) -> Result<()> {
        let gyro_config = GyroConfig::new().with_fs_sel(gyro_range);
        self.write(GYRO_CONFIG, &gyro_config.bytes).await?;
        Ok(())
    }

    pub async fn set_gyro_range_with_self_test(&mut self, gyro_range: GyroRange) -> Result<()> {
        let gyro_config = GyroConfig::new()
            .with_fs_sel(gyro_range)
            .with_xg_st(true)
            .with_yg_st(true)
            .with_zg_st(true);
        self.write(GYRO_CONFIG, &gyro_config.bytes).await?;
        Ok(())
    }

    pub async fn set_clock_source(&mut self, source: ClockSource) -> Result<()> {
        let pwr_mgmt1 = PwrMgmt1::new().with_clksel(source);
        self.write(PWR_MGMT_1, &pwr_mgmt1.bytes).await?;
        Ok(())
    }

    async fn write(&mut self, reg: u8, bits: &[u8; 1]) -> Result<()> {
        self.i2c.write_async(ADDR, [reg, bits[0]]).await?;
        Ok(())
    }

    async fn read(&mut self, reg: u8) -> Result<u8> {
        let mut data = [0; 1];
        self.i2c.write_read_async(ADDR, [reg], &mut data).await?;
        Ok(data[0])
    }
}