#![no_std]
#![no_main]
extern crate alloc;

mod mpu6050;
mod errors;

pub use panic_probe;
pub use defmt_rtt;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{Config, I2c, InterruptHandler};
use embassy_rp::peripherals::I2C1;
use crate::mpu6050::Mpu6050;
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let peripheral = embassy_rp::init(Default::default());
    let sda = peripheral.PIN_14;
    let scl = peripheral.PIN_15;
    let mut i2c = I2c::new_async(peripheral.I2C1, scl, sda, Irqs, Config::default());

    let mpu6050 = Mpu6050::new_async(i2c);
}