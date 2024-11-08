use alloc::format;
use alloc::string::String;
use core::fmt::Debug;
use embassy_rp::i2c::Error;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DroneError {
    #[error("I2C error: {0}")]
    I2c(String),
    #[error("invalid chip ID {0} (expected: {default})", default = crate::mpu6050::DEFAULT_SLAVE_ADDR)]
    InvalidChipId(u8)
}

impl From<embassy_rp::i2c::Error> for DroneError {
    fn from(value: Error) -> Self {
        DroneError::I2c(format!("{:?}", value))
    }
}

pub type Result<T> = core::result::Result<T, DroneError>;