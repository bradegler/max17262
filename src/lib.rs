//! # MAX17262 LiPo Fuel Guage
//!
//! The MAX17262 sensor utilizes the I2C interface.
//! This crate utilizes the embedded_hal constructs to provide a device
//! neutral implementation.
//!
//! See the main datasheet for this sensor [Data Sheet](https://download.siliconexpert.com/pdfs/2018/8/29/14/34/30/193383/max_/auto/max17262.pdf?#page=16&zoom=100,0,558)
//
#![no_std]

use core::marker::PhantomData;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub struct Max17262Config {
    pub charge_voltage: f32,
    pub design_cap: u16,
    pub ichg_term: u16,
    pub vempty: u16,
}

#[derive(Copy, Clone)]
#[allow(dead_code)]
enum Register {
    //I2cAddress = 0x6C,
    I2cAddress = 0x36,
    Por = 0x00,
    FstatDnr = 0x3D,
    HibCfg = 0xBA,
    HibCtrl = 0x60,
    DesignCap = 0x18,
    IchgTerm = 0x1E,
    Vempty = 0x3A,
    ModelCfg = 0xDB,
    RepCap = 0x05,
    RepSoC = 0x06,
    TTE = 0x11,
    RCOMP0 = 0x38,
    TempCo = 0x39,
    FullCapRep = 0x10,
    Cycles = 0x17,
    FullCapNom = 0x23,
}

const CHARGE_VOLTAGE_HIGH: u16 = 0x8400;
const CHARGE_VOLTAGE_LOW: u16 = 0x8000;

impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}
pub struct Max17262<I2C, Delay> {
    i2c: PhantomData<I2C>,
    delay: PhantomData<Delay>,
    recv_buffer: [u8; 2],
}

impl<I2C, Delay, E> Max17262<I2C, Delay>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    Delay: embedded_hal::blocking::delay::DelayMs<u8>,
    E: core::fmt::Debug,
{

    fn read(&mut self, i2c: &mut I2C, reg: u8) -> Result<u16, E> {
        match i2c.write_read(Register::I2cAddress.addr(), &[reg], &mut self.recv_buffer) {
            Ok(_) => Ok((self.recv_buffer[0] as u16) << 8 | self.recv_buffer[1] as u16),
            Err(e) => Err(e),
        }
    }

    fn write(&mut self, i2c: &mut I2C, reg: u8, value: u16) -> Result<(), E> {
        i2c.write(Register::I2cAddress.addr(), &[reg])?;
        let msb = ((value & 0xFF00) >> 8) as u8;
        let lsb = ((value & 0x00FF) >> 0) as u8;
        i2c.write(Register::I2cAddress.addr(), &[msb, lsb])?;
        Ok(())
    }

    fn config(&mut self, i2c: &mut I2C, delay: &mut Delay, cfg: Max17262Config) -> Result<(), E> {
        if self.read(i2c, Register::Por.addr()).unwrap() & 0x0002 == 0 {
            while self.read(i2c, Register::FstatDnr.addr()).unwrap() & 1 != 0 {
                // 10ms Wait Loop. Do not continue until FSTAT.DNR==0
                delay.delay_ms(10);
            }
            let hib_cfg = self.read(i2c, Register::HibCfg.addr()).unwrap(); //Store original HibCFG value
            self.write(i2c, Register::HibCtrl.addr(), 0x90)?; // Exit Hibernate Mode step 1
            self.write(i2c, Register::HibCfg.addr(), 0x0)?; // Exit Hibernate Mode step 2
            self.write(i2c, Register::HibCtrl.addr(), 0x0)?; // Exit Hibernate Mode step 3
            self.write(i2c, Register::DesignCap.addr(), cfg.design_cap)?;
            self.write(i2c, Register::IchgTerm.addr(), cfg.ichg_term)?;
            self.write(i2c, Register::Vempty.addr(), cfg.vempty)?;
            if cfg.charge_voltage > 4.275 {
                self.write(i2c, Register::ModelCfg.addr(), CHARGE_VOLTAGE_HIGH)?;
            } else {
                self.write(i2c, Register::ModelCfg.addr(), CHARGE_VOLTAGE_LOW)?;
            }
            //Poll ModelCFG.Refresh(highest bit),
            //proceed when ModelCFG.Refresh=0.
            while self.read(i2c, Register::ModelCfg.addr()).unwrap() & 0x8000 != 0 {
                delay.delay_ms(10);
            }
            //do not continue until ModelCFG.Refresh==0
            self.write(i2c, Register::HibCfg.addr(), hib_cfg)?; // Restore Original HibCFG value
        }
        // Clear the POR bit to indicate that the custom model and parameters were successfully loaded.
        let status = self.read(i2c, Register::Por.addr()).unwrap();
        self.write(i2c, Register::Por.addr(), status & 0xFFFD)?;
        Ok(())
    }

    pub fn new(i2c: &mut I2C, delay: &mut Delay, cfg: Max17262Config) -> Result<Self, E> {
        let mut max = Max17262 {
            i2c: PhantomData,
            delay: PhantomData,
            recv_buffer: [0u8; 2],
        };
        max.config(i2c, delay, cfg)?;
        Ok(max)
    }

    pub fn state_of_charge(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        self.read(i2c, Register::RepSoC.addr())
    }
}
