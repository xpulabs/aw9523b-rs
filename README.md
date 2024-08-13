# Rust AW9523B - 16 bit GPIO Explander & LED Driver's Driver

[crates.io](https://crates.io/crates/aw9523b)
[Docs](https://docs.rs/aw9523b)

This is a platform agnostic Rust driver for the aw9523b in handheld
and portable equipment using the [`embedded-hal`] traits.

This driver allows you to:
- Set GPIO input or output
- Set GPIO driver LED
- LED dimming
- Get GPIO status

The rust driver is tested with [AnalogLamb](https://www.analoglamb.com/) CiC AW9523B Breakout. You can buy the breakout from [AnalogLamb](https://www.analoglamb.com/) .

## The devices
AW9523B is a 16 multi-function LED driver and GPIO controller. Any of the 16 I/O ports can be configured as LED drive mode or GPIO mode.Furthermore, any GPIO can be configured as an input or an output independently.

After power on, all the 16 I/O ports are configured as GPIO output as default, which default states areset according to the I2C device address selectioninputs, AD0 and AD1. All I/O ports configured as inputs are continuously monitored for state changes. State changes are indicated by the INTN output. When AW9523B reads GPIO state through the I2C interface, the interrupt is cleared. Interrupt has 8μs deglitch.

When the I/O ports are configured as LED drive mode, AW9523B can set the current of LED drive between 0~IMAX by I2C interface, which is divided by 256 steps linear dimming. The default maximum current (IMAX) is 37mA, and IMAX can be changed in GCR register.

[Wiki - CIC AW9523B Breakout](https://xpulabs.github.io/products/amnos/interface/cic_ioe0001_aw9523b.html)


## Usage

To use this driver, import this crate and an `embedded_hal` implementation,
then instantiate the device.

Please find additional examples using hardware in CIC AW9523B Breakout
[Wiki - CIC AW9523B Breakout](https://xpulabs.github.io/products/amnos/interface/cic_ioe0001_aw9523b.html)

```rust
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
```

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in wiki FAQ](https://xpulabs.github.io/products/amnos/interface/cic_ioe0001_aw9523b.html#faq).

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.62 and up. It *might*
compile with older versions but that may change in any new patch release.

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   <http://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   <http://opensource.org/licenses/MIT>)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
