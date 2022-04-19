#![no_std]

use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::blocking::{InputPin,OutputPin};
use fugit::{
    ExtU32,
    Duration,
    NanosDurationU32,
}; //{MicrosDurationU32, MillisDurationU32};


pub enum Speed {
    Standard,
    Overdrive,
}

struct Speeds {
    A: NanosDurationU32,
    B: NanosDurationU32,
    C: NanosDurationU32,
    D: NanosDurationU32,
    E: NanosDurationU32,
    F: NanosDurationU32,
    G: NanosDurationU32,
    H: NanosDurationU32,
    I: NanosDurationU32,
    J: NanosDurationU32,
}

#[repr(u8)]
enum Command {
    SearchROM = 0xF0,
    ReadROM = 0x33,
    MatchROM = 0x55,
    SkipROM = 0xCC,
    AlarmSearch = 0xEC,
}

#[derive(Debug, Clone)]
pub struct ROM {
    family: u8,
    serial_number: [u8; 6],
    crc: u8,
}

#[derive(Debug, Copy, Clone)]
pub enum OneWireError<E> {
    BusNotHigh,
    Delay(E),
    Pin(E),
}

pub type OneWireResult<T, E> = Result<T, OneWireError<E>>;

pub struct OneWire<P, D> {
    pin: P,
    delay: D,
    speeds: Speeds,
    parasite_mode: bool,
}

impl<P, D, E> OneWire<P, D>
where
    P: InputPin<Error = E>,
    P: OutputPin<Error = E>,
    D: DelayUs<Error = E>,
{
    pub fn new(pin: P, delay: D, speed: Speed, parasite_mode: bool) -> OneWireResult<OneWire<P, D>, E> {
        let speeds = match speed {
            Speed::Standard =>
                Speeds {
                    A: NanosDurationU32::nanos(6000),
                    B: NanosDurationU32::nanos(64000),
                    C: NanosDurationU32::nanos(60000),
                    D: NanosDurationU32::nanos(10000),
                    E: NanosDurationU32::nanos(9000),
                    F: NanosDurationU32::nanos(55000),
                    G: NanosDurationU32::nanos(0),
                    H: NanosDurationU32::nanos(480000),
                    I: NanosDurationU32::nanos(70000),
                    J: NanosDurationU32::nanos(410000),
                },
            Speed::Overdrive =>
                Speeds {
                    A: NanosDurationU32::nanos(1000),
                    B: NanosDurationU32::nanos(7500),
                    C: NanosDurationU32::nanos(7500),
                    D: NanosDurationU32::nanos(2500),
                    E: NanosDurationU32::nanos(1000),
                    F: NanosDurationU32::nanos(7000),
                    G: NanosDurationU32::nanos(2500),
                    H: NanosDurationU32::nanos(70000),
                    I: NanosDurationU32::nanos(8500),
                    J: NanosDurationU32::nanos(40000),
                },
        };
        let mut wire = OneWire { pin, delay, speeds, parasite_mode };
        wire.release()?;
        Ok(wire)
    }

    pub fn write_1_bit(&mut self) -> OneWireResult<(), E> {
        self.set_low()?;
        self.delay.delay_us(self.speeds.A)?;
        self.release()?;
        self.delay.delay_us(self.speeds.B)?;
        Ok(())
    }

    pub fn write_0_bit(&mut self) -> OneWireResult<(), E> {
        self.set_low()?;
        self.delay.delay_us(self.speeds.C)?;
        self.release()?;
        self.delay.delay_us(self.speeds.D)?;
        Ok(())
    }

    pub fn read_bit(&mut self) -> OneWireResult<bool, E> {
        self.set_low()?;
        self.delay.delay_us(self.speeds.A)?;
        self.release()?;
        self.delay.delay_us(self.speeds.E)?;
        let bit = self.is_high()?;
        self.delay.delay_us(self.speeds.F)?;
        Ok(bit)
    }

    pub fn reset(&mut self) -> OneWireResult<bool, E> {
        self.wait_for_high()?;
        self.set_low()?;
        self.delay.delay_us(self.speeds.H)?;
        self.release()?;
        self.delay.delay_us(self.speeds.I)?;
        let devices_present = self.is_low()?;
        self.delay.delay_us(self.speeds.J)?;
        Ok(devices_present)
    }    

    // pub fn delay_us(&mut self, duration: MicrosDurationU32) -> OneWireResult<(), E> {
    //     self.delay
    //         .delay_us(duration.to_micros())
    //         .map_err(|e| OneWireError::Delay(e))
    // }

    // pub fn delay_ms(& mut self, duration: MillisDurationU32) -> OneWireResult<(), E> {
    //     self.delay
    //         .delay_ms(duration.to_millis())
    //         .map_err(|e| OneWireError::Delay(e))
    // }

    pub fn into_inner(self) -> P {
        self.pin
    }

    pub fn release(&mut self) -> OneWireResult<(), E> {
        self.pin
            .set_high()
            .map_err(|e| OneWireError::Pin(e))
    }

    pub fn set_low(&mut self) -> OneWireResult<(), E> {
        self.pin
            .set_low()
            .map_err(|e| OneWireError::Pin(e))
    }

    pub fn is_low(&self) -> OneWireResult<bool, E> {
        self.pin.is_low().map_err(|e| OneWireError::Pin(e))
    }

    pub fn is_high(&self) -> OneWireResult<bool, E> {
        self.pin.is_high().map_err(|e| OneWireError::Pin(e))
    }

    pub fn wait_for_high(&mut self) -> OneWireResult<(), E> {
        for _ in 0..125 {
            if self.is_high()? {
                self.delay.delay_us(self.speeds.G)?;
                return Ok(());
            }
            self.delay.delay_us(2)?;
        }
        Err(OneWireError::BusNotHigh)
    }

    pub fn write_bit(&mut self, bit: bool) -> OneWireResult<(), E> {
        if bit {
            self.write_1_bit()
        } else {
            self.write_0_bit()
        }
    }

    pub fn write_byte(&mut self, value: u8) -> OneWireResult<(), E> {
        for i in 0..8 {
            self.write_bit(value & 0x01 == 0x01)?;
            value >>= 1;
        }
        Ok(())
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) -> OneWireResult<(), E> {
        for i in 0..bytes.len() {
            self.write_byte(bytes[i])?;
        }
        Ok(())
    }
}