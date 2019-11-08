//! # MAX17262 LiPo Fuel Guage
//!
//! The MAX17262 sensor utilizes the I2C interface.
//! This crate utilizes the embedded_hal constructs to provide a device
//! neutral implementation.
//!
//! See the main datasheet for this sensor [Data Sheet](https://download.siliconexpert.com/pdfs/2018/8/29/14/34/30/193383/max_/auto/max17262.pdf?#page=16&zoom=100,0,558)
//
#![no_std]

pub mod max17262 {
    use embedded_hal::blocking::i2c::{Write, WriteRead};

    pub struct Max17262Config {
        pub charge_voltage: f32,
        pub design_cap: u16,
        pub ichg_term: u16,
        pub vempty: u16,
    }

    enum Registers {
        I2cAddress = 0x6C,
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

    pub struct Max17262<I> {
        i2c: I,
        recv_buffer: [u8; 2],
    }
    const CHARGE_VOLTAGE_HIGH: u16 = 0x8400;
    const CHARGE_VOLTAGE_LOW: u16 = 0x8000;

    impl<I, E> Max17262<I>
    where
        I: WriteRead<Error = E> + Write<Error = E>,
        E: core::fmt::Debug,
    {
        fn read(&mut self, reg: u8) -> Result<u16, E> {
            match self
                .i2c
                .write_read(Registers::I2cAddress as u8, &[reg], &mut self.recv_buffer)
            {
                Ok(_) => Ok((self.recv_buffer[0] as u16) << 8 | self.recv_buffer[1] as u16),
                Err(e) => Err(e),
            }
        }

        fn write(&mut self, reg: u8, value: u16) -> Result<(), E> {
            self.i2c.write(Registers::I2cAddress as u8, &[reg])?;
            let msb = ((value & 0xFF00) >> 8) as u8;
            let lsb = ((value & 0x00FF) >> 0) as u8;
            self.i2c.write(Registers::I2cAddress as u8, &[msb, lsb])?;
            Ok(())
        }

        fn wait(&mut self, _duration: u8) -> Result<(), E> {
            Ok(())
        }

        fn config(&mut self, cfg: Max17262Config) -> Result<(), E> {
            if self.read(Registers::Por as u8).unwrap() & 0x0002 == 0 {
                while self.read(Registers::FstatDnr as u8).unwrap() & 1 != 0 {
                    // 10ms Wait Loop. Do not continue until FSTAT.DNR==0
                    self.wait(10).ok();
                }
                let hib_cfg = self.read(Registers::HibCfg as u8).unwrap(); //Store original HibCFG value
                self.write(Registers::HibCtrl as u8, 0x90).unwrap(); // Exit Hibernate Mode step 1
                self.write(Registers::HibCfg as u8, 0x0).unwrap(); // Exit Hibernate Mode step 2
                self.write(Registers::HibCtrl as u8, 0x0).unwrap(); // Exit Hibernate Mode step 3
                self.write(Registers::DesignCap as u8, cfg.design_cap)
                    .unwrap();
                self.write(Registers::IchgTerm as u8, cfg.ichg_term)
                    .unwrap();
                self.write(Registers::Vempty as u8, cfg.vempty).unwrap();
                if cfg.charge_voltage > 4.275 {
                    self.write(Registers::ModelCfg as u8, CHARGE_VOLTAGE_HIGH)
                        .unwrap();
                } else {
                    self.write(Registers::ModelCfg as u8, CHARGE_VOLTAGE_LOW)
                        .unwrap();
                }
                //Poll ModelCFG.Refresh(highest bit),
                //proceed when ModelCFG.Refresh=0.
                while self.read(Registers::ModelCfg as u8).unwrap() & 0x8000 != 0 {
                    self.wait(10).unwrap();
                }
                //do not continue until ModelCFG.Refresh==0
                self.write(0xBA, hib_cfg).unwrap(); // Restore Original HibCFG value
            }
            // Clear the POR bit to indicate that the custom model and parameters were successfully loaded.
            let status = self.read(Registers::Por as u8).unwrap();
            self.write(Registers::Por as u8, status & 0xFFFD).unwrap();
            Ok(())
        }

        pub fn new(i2c: I, cfg: Max17262Config) -> Self {
            let mut max = Max17262 {
                i2c: i2c,
                recv_buffer: [0u8; 2],
            };
            max.config(cfg).unwrap();
            max
        }

        pub fn state_of_charge(&mut self) -> Result<u16, E> {
            self.read(Registers::RepSoC as u8)
        }
    }
}
