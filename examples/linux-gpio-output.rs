use linux_embedded_hal::I2cdev;
use aw9523b::Aw9523b;

fn main() {
    let dev = I2cdev::new("/dev/i2c-3").unwrap();
    let mut ic = Aw9523b::new(dev, 0x58);

    let id = ic.id().unwrap();
    println!("aw9523b id = {:02x}", id);
    std::thread::sleep(std::time::Duration::from_secs(1));

    ic.port0_output_mode(aw9523b::OutputMode::PP).unwrap();

    loop {
        println!("P00 set high");
        ic.set_pin_high(aw9523b::Pin::P00).unwrap();
        std::thread::sleep(std::time::Duration::from_secs(1));
        println!("P00 set low");
        ic.set_pin_low(aw9523b::Pin::P00).unwrap();
        std::thread::sleep(std::time::Duration::from_secs(1));

        println!("P10 set high");
        ic.set_pin_high(aw9523b::Pin::P10).unwrap();
        std::thread::sleep(std::time::Duration::from_secs(1));
        println!("P10 set low");
        ic.set_pin_low(aw9523b::Pin::P10).unwrap();
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}