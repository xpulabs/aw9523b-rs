#![no_std]

use embedded_hal::i2c;

#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum Register{
    INPUT_P0 = 0x00,
    INPUT_P1 = 0x01,
    OUTPUT_P0 = 0x02,
    OUTPUT_P1 = 0x03,
    CONFIG_P0 = 0x04,
    CONFIG_P1 = 0x05,
    INT_P0 = 0x06,
    INT_P1 = 0x07,
    ID = 0x10,
    CTL = 0x11,
    LEDMS_P0 = 0x12,
    LEDMS_P1 = 0x13,
    DIM0_P10 = 0x20,
    DIM1_P11 = 0x21,
    DIM2_P12 = 0x22,
    DIM3_P13 = 0x23,
    DIM4_P00 = 0x24,
    DIM5_P01 = 0x25,
    DIM6_P02 = 0x26,
    DIM7_P03 = 0x27,
    DIM8_P04 = 0x28,
    DIM9_P05 = 0x29,
    DIM10_P06 = 0x2A,
    DIM11_P07 = 0x2B,
    DIM12_P14 = 0x2C,
    DIM13_P15 = 0x2D,
    DIM14_P16 = 0x2E,
    DIM15_P17 = 0x2F,
    SW_RSTN = 0x7F,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(usize)]
pub enum Pin {
    P00 = 0,
    P01 = 1,
    P02 = 2,
    P03 = 3,
    P04 = 4,
    P05 = 5,
    P06 = 6,
    P07 = 7,
    P10 = 8,
    P11 = 9,
    P12 = 10,
    P13 = 11, 
    P14 = 12,
    P15 = 13,
    P16 = 14,
    P17 = 15,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(usize)]
pub enum Dir {
    INPUT = 1,
    OUTPUT = 0,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(usize)]
pub enum OutputMode {
    OD = 0,     // Open-Drain Mode
    PP = 1,     // Push-Pull Mode
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(usize)]
pub enum DimmingRange {
    IMAX = 0,           // Imax
    IMAX_3_4 = 1,       // Imax * 3/4
    IMAX_2_4 = 2,       // Imax * 2/4
    IMAX_1_4 = 3,       // Imax * 1/4
}

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// iic communication error
    I2C(E),
    /// invalid input data provided
    InvalidInputData,
}

#[derive(Debug)]
pub struct Aw9523b<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Aw9523b<I2C> 
where
    I2C: i2c::I2c<Error = E>
{
    /// Create new instance of the device
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Aw9523b {
            i2c,
            addr,
        }
    }

    /// Write Register 8bit Data
    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        let reg_val: [u8; 2] = [register as u8, value];
        self.i2c.write(self.addr, &reg_val).map_err(Error::I2C)
    }

    /// Read Register 8bit Data
    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut val = [0u8; 1];
        self.i2c.write_read(self.addr, &[register as u8], &mut val)
            .map_err(Error::I2C)
            .and(Ok(val[0]))
    }

    /// Read Chip ID
    pub fn id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::ID )
    }

    /// Set IO Direction
    pub fn set_io_direction(&mut self, pin: Pin, dir: Dir) -> Result<(), Error<E>> {
        let pin = pin as usize;
        let mut port = false;
        if pin & 8 != 0 {
            port = true;
        }

        let bit = pin & 0x7;

        let reg_config = if port {
            Register::CONFIG_P1
        } else {
            Register::CONFIG_P0
        };

        if dir == Dir::INPUT {
            let value = self.read_register(reg_config)?;
            self.write_register(reg_config, value | (0x1 << bit))
        } else {
            let value = self.read_register(reg_config)?;
            self.write_register(reg_config, value & !(0x1 << bit))
        }
    }

    /// Set Pin Output High
    pub fn set_pin_high(&mut self, pin: Pin) -> Result<(), Error<E>> {
        let pin = pin as usize;
        let mut port = false;
        if pin & 8 != 0 {
            port = true;
        }

        let bit = pin & 0x7;

        let (reg_config, reg_output) = match port {
            false => (Register::CONFIG_P0, Register::OUTPUT_P0),
            true => (Register::CONFIG_P1, Register::OUTPUT_P1),
        };

        let value = self.read_register(reg_config)?;
        self.write_register(reg_config, value & !(0x1 << bit))?;
        let value = self.read_register(reg_output)?;
        self.write_register(reg_output, value | (0x1 << bit))
    }

    /// Set Pin Output Low
    pub fn set_pin_low(&mut self, pin: Pin) -> Result<(), Error<E>> {
        let pin = pin as usize;
        let mut port = false;   //P0
        if pin & 0x08 != 0 {
            port = true;
        }

        let bit = pin & 0x7;
        let (reg_config, reg_output) = match port {
            false => (Register::CONFIG_P0, Register::OUTPUT_P0),
            true => (Register::CONFIG_P1, Register::OUTPUT_P1),
        };

        let value = self.read_register(reg_config)?;
        self.write_register(reg_config, value & !(0x1 << bit))?;
        let value = self.read_register(reg_output)?;
        self.write_register(reg_output, value & !(0x1 << bit))
    }

    pub fn pin_is_high(&mut self, pin: Pin) -> Result<bool, Error<E>> {
        let pin = pin as usize;
        let mut port = false;   // P0
        if pin & 8 != 0 {
            port = true;    // P1
        }

        let bit = pin & 0x7;

        let reg_input = if port {
            Register::INPUT_P1
        } else {
            Register::INPUT_P0
        };

        let value = self.read_register(reg_input)?;
        match (value >> bit) & 1 {
            1 => Ok(true),
            _ => Ok(false),
        }
    }

    pub fn pin_is_low(&mut self, pin: Pin) -> Result<bool, Error<E>> {
        let pin = pin as usize;
        let mut port = false;   // P0
        if pin & 8 != 0 {
            port = true;    // P1
        }

        let bit = pin & 0x7;

        let reg_input = if port {
            Register::INPUT_P1
        } else {
            Register::INPUT_P0
        };

        let value = self.read_register(reg_input)?;
        match (value >> bit) & 1 {
            1 => Ok(false),
            _ => Ok(true),
        }
    }

    pub fn pin_enable_interrupt(&mut self, pin: Pin, en: bool) -> Result<(), Error<E>> {

        let pin = pin as usize;
        let mut port = false;   // P0
        if pin & 8 != 0 {
            port = true;    // P1
        }

        let bit = pin & 0x7;

        let reg = if port {
            Register::INT_P1
        } else {
            Register::INT_P0
        };

        if en {
            let value = self.read_register(reg)?;
            self.write_register(reg, value & !(0x1 << bit))
        } else {
            let value = self.read_register(reg)?;
            self.write_register(reg, value | (0x1 << bit))
        }
    }

    pub fn pin_gpio_mode(&mut self, pin: Pin) -> Result<(), Error<E>> {
        let pin = pin as usize;
        let mut port = false;   // P0
        if pin & 8 != 0 {
            port = true;    // P1
        }

        let bit = pin & 0x7;

        let reg = if port {
            Register::LEDMS_P1
        } else {
            Register::LEDMS_P0
        };

        let value = self.read_register(reg)?;
        self.write_register(reg, value | (0x1 << bit))
    }

    pub fn pin_led_mode(&mut self, pin: Pin) -> Result<(), Error<E>> {
        let pin = pin as usize;
        let mut port = false;   // P0
        if pin & 8 != 0 {
            port = true;    // P1
        }

        let bit = pin & 0x7;

        let reg = if port {
            Register::LEDMS_P1
        } else {
            Register::LEDMS_P0
        };

        let value = self.read_register(reg)?;
        self.write_register(reg, value & !(0x1 << bit))
    }

    pub fn port0_output_mode(&mut self, mode: OutputMode) -> Result<(), Error<E>> {

        let value = self.read_register(Register::CTL)?;
        let value = match mode {
            OutputMode::OD => value & !(0x1 << 4),
            OutputMode::PP => value | (0x1 << 4),
        };
        self.write_register(Register::CTL, value)
    }

    pub fn led_dimming_range(&mut self, range: DimmingRange) -> Result<(), Error<E>> {
        let value = self.read_register(Register::CTL)?;
        let value = match range {
            DimmingRange::IMAX => value & !(0x3),
            DimmingRange::IMAX_3_4 => (value & !(0x3)) | 0x1,
            DimmingRange::IMAX_2_4 => (value & !(0x3)) | 0x2,
            DimmingRange::IMAX_1_4 => value | 0x3,
        };

        self.write_register(Register::CTL, value)
    }

    pub fn led_set_dimming(&mut self, pin: Pin, dimming: u8) -> Result<(), Error<E>> {
        let reg = match pin {
            Pin::P00 => Register::DIM4_P00,
            Pin::P01 => Register::DIM5_P01,
            Pin::P02 => Register::DIM6_P02,
            Pin::P03 => Register::DIM7_P03,
            Pin::P04 => Register::DIM8_P04,
            Pin::P05 => Register::DIM9_P05,
            Pin::P06 => Register::DIM10_P06,
            Pin::P07 => Register::DIM11_P07,
            Pin::P10 => Register::DIM0_P10,
            Pin::P11 => Register::DIM1_P11,
            Pin::P12 => Register::DIM2_P12,
            Pin::P13 => Register::DIM3_P13,
            Pin::P14 => Register::DIM12_P14,
            Pin::P15 => Register::DIM13_P15,
            Pin::P16 => Register::DIM14_P16,
            Pin::P17 => Register::DIM15_P17,
        };

        self.write_register(reg, dimming)
    }

    pub fn software_reset(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::SW_RSTN, 0x00)
    }
}

