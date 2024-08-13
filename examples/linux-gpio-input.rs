use linux_embedded_hal::I2cdev;
use aw9523b::Aw9523b;

fn main() {
    let dev = I2cdev::new("/dev/i2c-3").unwrap();
    let mut ic = Aw9523b::new(dev, 0x58);

    let id = ic.id().unwrap();
    println!("aw9523b id = {:02x}", id);
    std::thread::sleep(std::time::Duration::from_secs(1));

    ic.set_io_direction(aw9523b::Pin::P00, aw9523b::Dir::INPUT).unwrap();

    loop {
        while !ic.pin_is_low(aw9523b::Pin::P00).unwrap() {
            println!("P00 is high");
            std::thread::sleep(std::time::Duration::from_millis(200));
        }
        println!("P00 is low");
    }
}