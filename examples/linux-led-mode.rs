use linux_embedded_hal::I2cdev;
use aw9523b::Aw9523b;

fn main() {
    let dev = I2cdev::new("/dev/i2c-3").unwrap();
    let mut ic = Aw9523b::new(dev, 0x58);

    let id = ic.id().unwrap();
    println!("aw9523b id = {:02x}", id);
    std::thread::sleep(std::time::Duration::from_secs(1));

    println!("aw9523b led mode: dimming");
    ic.pin_led_mode(aw9523b::Pin::P00).unwrap();
    ic.pin_led_mode(aw9523b::Pin::P17).unwrap();

    ic.led_dimming_range(aw9523b::DimmingRange::IMAX).unwrap();

    loop {

        for i in 0u8..255 {
            ic.led_set_dimming(aw9523b::Pin::P00, i).unwrap();
            ic.led_set_dimming(aw9523b::Pin::P17, 255-i).unwrap();
            if i < 200 {
                std::thread::sleep(std::time::Duration::from_millis(10));
            } else {
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
            
        }
    }
}