#![no_std]
//! A `no_std` compatible driver for the RX8900 real-time clock (RTC) chip, intended for use in embedded systems where no standard library is available.

use heapless::Vec; // Provides a fixed-size vector data structure.
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike, Weekday}; // Provides date and time utility types.
use embedded_hal::blocking::i2c::{Read, Write, WriteRead}; // Traits for blocking I2C communication.

const RX8900_ADDR: u8 = 0x32; // I2C address for the RX8900 RTC device.

/// Defines the register map for the RX8900 RTC device. Includes both standard and extended registers.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
enum RegisterTable {
    // Standard registers compatible with RX-8803
    CompatibleSEC = 0x00, // Register for seconds
    CompatibleMIN = 0x01, // Register for minutes
    CompatibleHOUR = 0x02, // Register for hours
    CompatibleWEEK = 0x03, // Register for the day of the week
    CompatibleDAY = 0x04, // Register for the day of the month
    CompatibleMONTH = 0x05, // Register for the month
    CompatibleYEAR = 0x06, // Register for the year
    CompatibleRAM = 0x07, // General purpose RAM
    CompatibleMinAlarm = 0x08, // Minutes alarm register
    CompatibleHourAlarm = 0x09, // Hours alarm register
    CompatibleWeekDayAlarm = 0x0A, // Day of the week alarm register
    CompatibleTimerCounter0 = 0x0B, // Lower byte of timer/counter
    CompatibleTimerCounter1 = 0x0C, // Upper byte of timer/counter
    CompatibleExtensionRegister = 0x0D, // Extension register for additional settings
    CompatibleFlagRegister = 0x0E, // Flag register indicating various statuses
    CompatibleControlRegister = 0x0F, // Control register for various configurations

    // Extended registers
    ExtendedSEC = 0x10, // Extended register for seconds
    ExtendedMIN = 0x11, // Extended register for minutes
    ExtendedHOUR = 0x12, // Extended register for hours
    ExtendedWEEK = 0x13, // Extended register for the day of the week
    ExtendedDAY = 0x14, // Extended register for the day of the month
    ExtendedMONTH = 0x15, // Extended register for the month
    ExtendedYEAR = 0x16, // Extended register for the year
    ExtendedTEMP = 0x17, // Temperature sensor output
    ExtendedBackupFunction = 0x18, // Backup functionality control register
    ExtendedTimerCounter0 = 0x1B, // Extended lower byte of timer/counter
    ExtendedTimerCounter1 = 0x1C, // Extended upper byte of timer/counter
    ExtendedExtensionRegister = 0x1D, // Extended extension register
    ExtendedFlagRegister = 0x1E, // Extended flag register
    ExtendedControlRegister = 0x1F, // Extended control register
}

/// Represents the possible clock sources for triggering events.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SourceClock {
    SourceClock4096Hz = 0b00,
    SourceClock64Hz = 0b01,
    SourceClockSecond = 0b10,
    SourceClockMinute = 0b11,
}

/// Represents different types of alarms that can be configured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AlarmType {
    WeekAlarm = 0b00,
    DayAlarm = 0b01,
}

/// Represents the frequency at which update interrupts are generated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UpdateInterruptType {
    EverySecond = 0b00,
    EveryMinute = 0b01,
}

/// Represents the frequency of the fout (frequency out) pin output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FoutFrequency {
    FoutFrequency32_768kHz = 0b00,
    FoutFrequency1024Hz = 0b01,
    FoutFrequency1Hz = 0b10,
    // FoutFrequency32_768kHz = 0b11,
}

/// Represents the intervals at which temperature compensation is applied.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CompensationIntervalType {
    CompensationInterval0_5s = 0b00,
    CompensationInterval2_0s = 0b01,
    CompensationInterval10s = 0b10,
    CompensationInterval30s = 0b11,
}

/// A struct representing the RX8900 RTC device interfaced over I2C.
pub struct Rx8900<I2C> {
    i2c: I2C,
}

impl<I2C> Rx8900<I2C> {
    /// Creates a new instance of the Rx8900 RTC device driver.
    ///
    /// # Arguments
    /// * `i2c` - An instance of the I2C peripheral to communicate with the RX8900.
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, }
    }

    /// Converts a BCD-encoded byte to a regular decimal byte.
    fn from_bcd(data: u8) -> u8 {
        (data >> 4) * 10 + (data & 0x0F)
    }

    /// Converts a regular decimal byte to a BCD-encoded byte.
    fn to_bcd(data: u8) -> u8 {
        ((data / 10) << 4) | (data % 10)
    }

    /// Converts a numerical representation of a weekday into a `Weekday` enum.
    fn from_week(data: u8) -> Weekday {
        match data {
            0b00000001 => Weekday::Sun,
            0b00000010 => Weekday::Mon,
            0b00000100 => Weekday::Tue,
            0b00001000 => Weekday::Wed,
            0b00010000 => Weekday::Thu,
            0b00100000 => Weekday::Fri,
            0b01000000 => Weekday::Sat,
            _ => todo!(), // placeholder for handling invalid data
        }
    }

    /// Converts a `Weekday` enum into its numerical representation.
    fn to_week(data: Weekday) -> u8 {
        match data {
            Weekday::Sun => 0b00000001,
            Weekday::Mon => 0b00000010,
            Weekday::Tue => 0b00000100,
            Weekday::Wed => 0b00001000,
            Weekday::Thu => 0b00010000,
            Weekday::Fri => 0b00100000,
            Weekday::Sat => 0b01000000,
        }
    }
}

impl<I2C, E> Rx8900<I2C>
where
    I2C: Read<Error = E> + WriteRead<Error = E>,
{
    /// Reads a single byte from a specified register.
    ///
    /// # Arguments
    /// * `register` - The register from which to read.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The read byte on success or an error if the read fails.
    fn read_register(&mut self, register: RegisterTable) -> Result<u8, E> {
        let mut data = [0];
        self.i2c.write_read(RX8900_ADDR, &[register as u8], &mut data)?;
        Ok(data[0])
    }

    /// Reads a specific bit from a register and returns it as a boolean value.
    ///
    /// # Arguments
    /// * `register` - The register from which to read.
    /// * `bit` - The bit position to read (0-7).
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the bit is set, false otherwise, or an error if the read fails.
    fn read_register_1bit(&mut self, register: RegisterTable, bit: u8) -> Result<bool, E> {
        let data = self.read_register(register)?;
        Ok((data & (1 << bit)) == (1 << bit))
    }

    /// Returns the current second value from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current second, or an error if the read fails.
    pub fn sec(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleSEC)?;
        Ok(Self::from_bcd(data & 0b01111111))
    }

    /// Returns the current minute value from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current minute, or an error if the read fails.
    pub fn min(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleMIN)?;
        Ok(Self::from_bcd(data & 0b01111111))
    }

    /// Returns the current hour value from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current hour, or an error if the read fails.
    pub fn hour(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleHOUR)?;
        Ok(Self::from_bcd(data & 0b00111111))
    }

    /// Returns the current day of the week from the RTC.
    ///
    /// # Returns
    /// * `Result<Weekday, E>` - The current weekday, or an error if the read fails.
    pub fn week(&mut self) -> Result<Weekday, E> {
        let data = self.read_register(RegisterTable::CompatibleWEEK)?;
        Ok(Self::from_week(data))
    }

    /// Returns the current day of the month from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current day, or an error if the read fails.
    pub fn day(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleDAY)?;
        Ok(Self::from_bcd(data & 0b00111111))
    }

    /// Returns the current month from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current month, or an error if the read fails.
    pub fn month(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleMONTH)?;
        Ok(Self::from_bcd(data & 0b00011111))
    }

    /// Returns the current year from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The current year, or an error if the read fails.
    pub fn year(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleYEAR)?;
        Ok(Self::from_bcd(data))
    }

    /// Reads the general-purpose RAM value from the RTC.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The RAM content, or an error if the read fails.
    pub fn ram(&mut self) -> Result<u8, E> {
        self.read_register(RegisterTable::CompatibleRAM)
    }

    /// Reads the minute alarm value and checks if the alarm is enabled.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The minute for the alarm, or an error if the read fails.
    pub fn min_alarm(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleMinAlarm)?;
        Ok(Self::from_bcd(data & 0b01111111))
    }

    /// Checks if the minute alarm is currently enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the minute alarm is enabled, or an error if the read fails.
    pub fn min_alarm_enabled(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleMinAlarm, 7)
    }

    /// Reads the hour alarm value and checks if the alarm is enabled.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The hour for the alarm, or an error if the read fails.
    pub fn hour_alarm(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleHourAlarm)?;
        Ok(Self::from_bcd(data & 0b00111111))
    }

    /// Checks if the hour alarm is currently enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the hour alarm is enabled, or an error if the read fails.
    pub fn hour_alarm_enabled(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleHourAlarm, 7)
    }

    /// Retrieves the current set weekdays for the week alarm.
    ///
    /// # Returns
    /// * `Result<Vec<Weekday, 7>, E>` - A vector of `Weekday` representing the days set in the week alarm, or an error if the read fails.
    pub fn week_alarm(&mut self) -> Result<Vec<Weekday, 7>, E> {
        let mut weekdays = Vec::<Weekday, 7>::new();
        let data = self.read_register(RegisterTable::CompatibleWeekDayAlarm)?;
        for i in 0..7 {
            let bit = 1 << i;
            if (data & bit) == bit {
                weekdays.push(Self::from_week(bit)).unwrap();
            }
        }
        Ok(weekdays)
    }

    /// Checks if the week alarm is currently enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the week alarm is enabled, or an error if the read fails.
    pub fn week_alarm_enabled(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleWeekDayAlarm, 7)
    }

    /// Retrieves the day for the day alarm.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The day in the day alarm, or an error if the read fails.
    pub fn day_alarm(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleWeekDayAlarm)?;
        Ok(Self::from_bcd(data & 0b01111111))
    }

    /// Checks if the day alarm is currently enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the day alarm is enabled, or an error if the read fails.
    pub fn day_alarm_enabled(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleWeekDayAlarm, 7)
    }

    /// Reads the value of the timer counter 0.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The value of timer counter 0, or an error if the read fails.
    pub fn timer_counter0(&mut self) -> Result<u8, E> {
        self.read_register(RegisterTable::CompatibleTimerCounter0)
    }

    /// Reads the value of the timer counter 1.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The value of timer counter 1, or an error if the read fails.
    pub fn timer_counter1(&mut self) -> Result<u8, E> {
        self.read_register(RegisterTable::CompatibleTimerCounter1)
    }
    
    /// Combines the values of timer counter 0 and timer counter 1 into a single 16-bit value.
    ///
    /// # Returns
    /// * `Result<u16, E>` - The combined value of the two timer counters, or an error if the read fails.
    pub fn timer_counter(&mut self) -> Result<u16, E> {
        let data0 = self.read_register(RegisterTable::CompatibleTimerCounter0)?;
        let data1 = self.read_register(RegisterTable::CompatibleTimerCounter1)?;
        Ok((data1 as u16) << 8 | data0 as u16)
    }

    /// Reads the current temperature value from the RTC's temperature sensor.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The raw temperature value, or an error if the read fails.
    pub fn temp(&mut self) -> Result<u8, E> {
        self.read_register(RegisterTable::ExtendedTEMP)
    }

    /// Converts the raw temperature value to Celsius.
    ///
    /// # Returns
    /// * `Result<f32, E>` - The temperature in Celsius, or an error if the read fails.
    pub fn temp_in_cercius(&mut self) -> Result<f32, E> {
        let data = self.temp()?;
        Ok((data as f32 * 2.0 - 187.19) / 3.218)
    }

    /// Checks if the voltage detector is currently turned off.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the voltage detector is off, or an error if the read fails.
    pub fn vdetoff(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::ExtendedBackupFunction, 3)
    }

    /// Alias for `vdetoff`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the voltage detector is off, or an error if the read fails.
    pub fn voltage_detector_off(&mut self) -> Result<bool, E> {
        self.vdetoff()
    }

    /// Checks if the switch off control is currently enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the switch off control is enabled, or an error if the read fails.
    pub fn swoff(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::ExtendedBackupFunction, 2)
    }

    /// Alias for `swoff`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the switch off control is enabled, or an error if the read fails.
    pub fn switch_off(&mut self) -> Result<bool, E> {
        self.swoff()
    }

    /// Checks if backup mode sample bit 1 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the backup mode sample bit 1 is set, or an error if the read fails.
    pub fn bksmp1(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::ExtendedBackupFunction, 1)
    }

    /// Checks if backup mode sample bit 0 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the backup mode sample bit 0 is set, or an error if the read fails.
    pub fn bksmp0(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::ExtendedBackupFunction, 0)
    }

    /// Reads the backup mode sampling time configuration.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The backup mode sampling time configuration value, or an error if the read fails.
    pub fn bksmp(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::ExtendedBackupFunction)?;
        Ok((data & 0b00000011) >> 0)
    }

    /// Alias for `bksmp`.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The backup mode sampling time configuration value, or an error if the read fails.
    pub fn backup_mode_sampling_time(&mut self) -> Result<u8, E> {
        self.bksmp()
    }

    /// Determines the type of alarm currently set.
    ///
    /// # Returns
    /// * `Result<AlarmType, E>` - The type of alarm (day or week), or an error if the read fails.
    pub fn alarm_type(&mut self) -> Result<AlarmType, E> {
        self.wada().map(|x| if x { AlarmType::DayAlarm } else { AlarmType::WeekAlarm })
    }

    /// Performs a test by reading a test bit, typically used for diagnostics.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the test bit is set (indicating some test condition), or an error if the read fails.
    pub fn test(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 7)
    }

    /// Determines the alarm day/week setting (WADA bit).
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the WADA bit is set (day alarm), false if cleared (week alarm), or an error if the read fails.
    pub fn wada(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 6)
    }

    /// Retrieves the current setting for update interrupt type.
    ///
    /// # Returns
    /// * `Result<UpdateInterruptType, E>` - The update interrupt type (every second or minute), or an error if the read fails.
    pub fn update_interrupt_type(&mut self) -> Result<UpdateInterruptType, E> {
        self.usel().map(|x| if x { UpdateInterruptType::EveryMinute } else { UpdateInterruptType::EverySecond })
    }

    /// Determines the update selection bit for controlling the interrupt frequency.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the update selection is set for minute, false for second, or an error if the read fails.
    pub fn usel(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 5)
    }

    /// Checks if the timer is enabled.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer is enabled, or an error if the read fails.
    pub fn te(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 4)
    }

    /// Alias for `te`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer is enabled, or an error if the read fails.
    pub fn timer_enable(&mut self) -> Result<bool, E> {
        self.te()
    }

    /// Checks if fout frequency selection bit 1 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the fout frequency selection bit 1 is set, or an error if the read fails.
    pub fn fsel1(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 3)
    }

    /// Checks if fout frequency selection bit 0 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the fout frequency selection bit 0 is set, or an error if the read fails.
    pub fn fsel0(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 2)
    }

    /// Reads the fout frequency selection setting from the extension register.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The fout frequency selection bits, or an error if the read fails.
    pub fn fsel(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        Ok((data & 0b00001100) >> 2)
    }

    /// Reads the fout frequency selection bits and returns the corresponding fout frequency.
    ///
    /// # Returns
    /// * `Result<FoutFrequency, E>` - The fout frequency setting, or an error if the read fails.
    pub fn fout_frequency(&mut self) -> Result<FoutFrequency, E> {
        let current = self.fsel()?;
        Ok(match current {
            0b00 => FoutFrequency::FoutFrequency32_768kHz,
            0b01 => FoutFrequency::FoutFrequency1024Hz,
            0b10 => FoutFrequency::FoutFrequency1Hz,
            0b11 => FoutFrequency::FoutFrequency32_768kHz,
            _ => todo!(),
        })
    }

    /// Retrieves the current source clock setting.
    ///
    /// # Returns
    /// * `Result<SourceClock, E>` - The current source clock setting, or an error if the read fails.
    pub fn source_clock(&mut self) -> Result<SourceClock, E> {
        let data = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        Ok(match (data & 0b00000011) >> 0 {
            0b00 => SourceClock::SourceClock4096Hz,
            0b01 => SourceClock::SourceClock64Hz,
            0b10 => SourceClock::SourceClockSecond,
            0b11 => SourceClock::SourceClockMinute,
            _ => todo!(),
        })
    }
    
    /// Checks if timer selection bit 1 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer selection bit 1 is set, or an error if the read fails.
    pub fn tsel1(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 1)
    }

    /// Checks if timer selection bit 0 is set.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer selection bit 0 is set, or an error if the read fails.
    pub fn tsel0(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleExtensionRegister, 0)
    }

    /// Reads the timer selection bits.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The timer selection configuration value, or an error if the read fails.
    pub fn tsel(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        Ok((data & 0b00000011) >> 0)
    }

    /// Reads the update flag which indicates if an update occurred.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if an update has occurred, otherwise false, or an error if the read fails.
    pub fn uf(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleFlagRegister, 5)
    }

    /// Alias for `uf`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if an update has occurred, false otherwise, or an error if the read fails.
    pub fn update_flag(&mut self) -> Result<bool, E> {
        self.uf()
    }

    /// Reads the timer flag which indicates if a timer event has occurred.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if a timer event has occurred, otherwise false, or an error if the read fails.
    pub fn tf(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleFlagRegister, 4)
    }

    /// Alias for `tf`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if a timer interrupt has occurred, false otherwise, or an error if the read fails.
    pub fn timer_flag(&mut self) -> Result<bool, E> {
        self.tf()
    }

    /// Reads the alarm flag which indicates if an alarm has been triggered.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if an alarm has been triggered, otherwise false, or an error if the read fails.
    pub fn af(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleFlagRegister, 3)
    }

    /// Alias for `af`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if an alarm has been triggered, false otherwise, or an error if the read fails.
    pub fn alarm_flag(&mut self) -> Result<bool, E> {
        self.af()
    }

    /// Reads the voltage low flag which indicates if the battery voltage has dropped below a critical threshold.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the voltage is low, otherwise false, or an error if the read fails.
    pub fn vlf(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleFlagRegister, 1)
    }

    /// Alias for `vlf`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the voltage is low, false otherwise, or an error if the read fails.
    pub fn voltage_low_flag(&mut self) -> Result<bool, E> {
        self.vlf()
    }

    /// Reads the voltage detect flag which indicates if a voltage drop has been detected.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if a voltage drop has been detected, otherwise false, or an error if the read fails.
    pub fn vdet(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleFlagRegister, 0)
    }

    /// Alias for `vdet`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if a voltage drop has been detected, false otherwise, or an error if the read fails.
    pub fn voltage_detect_flag(&mut self) -> Result<bool, E> {
        self.vdet()
    }

    /// Reads the first compensation interval selection bit.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the first compensation interval bit is set, or an error if the read fails.
    pub fn csel1(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 7)
    }

    /// Reads the second compensation interval selection bit.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the second compensation interval bit is set, or an error if the read fails.
    pub fn csel0(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 6)
    }

    /// Reads the control selection bits to determine the current compensation interval configuration.
    ///
    /// # Returns
    /// * `Result<u8, E>` - The control selection value, or an error if the read fails.
    pub fn csel(&mut self) -> Result<u8, E> {
        let data = self.read_register(RegisterTable::CompatibleControlRegister)?;
        Ok((data & 0b11000000) >> 6)
    }

    /// Reads the compensation interval type from the control register.
    ///
    /// # Returns
    /// * `Result<CompensationIntervalType, E>` - The current compensation interval setting, or an error if the read fails.
    pub fn compensation_interval_type(&mut self) -> Result<CompensationIntervalType, E> {
        let current = self.csel()?;
        Ok(match current {
            0b00 => CompensationIntervalType::CompensationInterval0_5s,
            0b01 => CompensationIntervalType::CompensationInterval2_0s,
            0b10 => CompensationIntervalType::CompensationInterval10s,
            0b11 => CompensationIntervalType::CompensationInterval30s,
            _ => todo!(),
        })
    }

    /// Reads the status of the update interrupt enable bit.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the update interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn uie(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 5)
    }

    /// Alias for `uie`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the update interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn update_interrupt_enable(&mut self) -> Result<bool, E> {
        self.uie()
    }

    /// Reads the status of the timer interrupt enable bit.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn tie(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 4)
    }

    /// Alias for `tie`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the timer interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn timer_interrupt_enable(&mut self) -> Result<bool, E> {
        self.tie()
    }

    /// Reads the status of the alarm interrupt enable bit.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the alarm interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn aie(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 3)
    }

    /// Alias for `tie`.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if the alarm interrupt is enabled, otherwise false, or an error if the read fails.
    pub fn alarm_interrupt_enable(&mut self) -> Result<bool, E> {
        self.aie()
    }

    /// Reads the reset bit to determine if a reset operation has been triggered.
    ///
    /// # Returns
    /// * `Result<bool, E>` - True if a reset has been triggered, otherwise false, or an error if the read fails.
    pub fn reset(&mut self) -> Result<bool, E> {
        self.read_register_1bit(RegisterTable::CompatibleControlRegister, 0)
    }
}

impl<I2C, E> Rx8900<I2C>
where
    I2C: Write<Error = E>,
{
    /// Writes a single byte to a specified register.
    ///
    /// # Arguments
    /// * `register` - The register to which the byte will be written.
    /// * `data` - The data byte to write.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the write was successful, or an error if the write fails.
    fn write_register(&mut self, register: RegisterTable, data: u8) -> Result<(), E> {
        self.i2c.write(RX8900_ADDR, &[register as u8, data])?;
        Ok(())
    }

    /// Sets the second value in the RTC.
    ///
    /// # Arguments
    /// * `data` - The second to be set (0-59).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the second was successfully set, or an error if the write fails.
    pub fn set_sec(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleSEC, Self::to_bcd(data & 0b01111111))
    }

    /// Sets the minute value in the RTC.
    ///
    /// # Arguments
    /// * `data` - The minute to be set (0-59).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the minute was successfully set, or an error if the write fails.
    pub fn set_min(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleMIN, Self::to_bcd(data & 0b01111111))
    }

    /// Sets the hour value in the RTC.
    ///
    /// # Arguments
    /// * `data` - The hour to be set (0-23).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the hour was successfully set, or an error if the write fails.
    pub fn set_hour(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleHOUR, Self::to_bcd(data & 0b00111111))
    }

    /// Sets the day of the week in the RTC.
    ///
    /// # Arguments
    /// * `data` - The `Weekday` to be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the weekday was successfully set, or an error if the write fails.
    pub fn set_week(&mut self, data: Weekday) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleWEEK, Self::to_week(data))
    }

    /// Sets the day of the month in the RTC.
    ///
    /// # Arguments
    /// * `data` - The day to be set (1-31).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the day was successfully set, or an error if the write fails.
    pub fn set_day(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleDAY, Self::to_bcd(data & 0b00111111))
    }

    /// Sets the month in the RTC.
    ///
    /// # Arguments
    /// * `data` - The month to be set (1-12).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the month was successfully set, or an error if the write fails.
    pub fn set_month(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleMONTH, Self::to_bcd(data & 0b00011111))
    }

    /// Sets the year in the RTC.
    ///
    /// # Arguments
    /// * `data` - The year to be set (0-99).
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the year was successfully set, or an error if the write fails.
    pub fn set_year(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleYEAR, Self::to_bcd(data & 0b11111111))
    }

    /// Writes a byte to the general-purpose RAM in the RTC.
    ///
    /// # Arguments
    /// * `data` - The byte to be written to RAM.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the write was successful, or an error if the write fails.
    pub fn set_ram(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleRAM, data)
    }

    /// Enables or disables the minute alarm.
    ///
    /// # Arguments
    /// * `data` - The minute to set for the alarm.
    /// * `enabled` - True to enable the alarm, false to disable.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm setting was successful, or an error if the write fails.
    pub fn set_min_alarm(&mut self, data: u8, enabled: bool) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleMinAlarm, Self::to_bcd(data & 0b01111111) | (enabled as u8) << 7)
    }

    /// Enables or disables the hour alarm.
    ///
    /// # Arguments
    /// * `data` - The hour to set for the alarm.
    /// * `enabled` - True to enable the alarm, false to disable.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm setting was successful, or an error if the write fails.
    pub fn set_hour_alarm(&mut self, data: u8, enabled: bool) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleHourAlarm, Self::to_bcd(data & 0b00111111) | (enabled as u8) << 7)
    }

    /// Sets the week alarm for the specified weekdays.
    ///
    /// # Arguments
    /// * `data` - A reference to a vector containing the weekdays for which the alarm should be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm was successfully set, or an error if the operation fails.
    pub fn set_week_alarm(&mut self, data: &Vec<Weekday, 7>) -> Result<(), E> {
        let mut value = 0;
        for day in data {
            value |= Self::to_week(*day);
        }
        self.write_register(RegisterTable::CompatibleWeekDayAlarm, value)
    }

    /// Sets the day alarm value and enables or disables it.
    ///
    /// # Arguments
    /// * `data` - The day value for the alarm.
    /// * `enabled` - True to enable the alarm, false to disable it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm was successfully set, or an error if the operation fails.
    pub fn set_day_alarm(&mut self, data: u8, enabled: bool) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleWeekDayAlarm, Self::to_bcd(data & 0b01111111) | (enabled as u8) << 7)
    }

    /// Sets the value of timer counter 0.
    ///
    /// # Arguments
    /// * `data` - The value to set for timer counter 0.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the value was successfully set, or an error if the write fails.
    pub fn set_timer_counter0(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleTimerCounter0, data)
    }

    /// Sets the value of timer counter 1.
    ///
    /// # Arguments
    /// * `data` - The value to set for timer counter 1.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the value was successfully set, or an error if the write fails.
    pub fn set_timer_counter1(&mut self, data: u8) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleTimerCounter1, data)
    }

    /// Sets the combined value of timer counter 0 and timer counter 1.
    ///
    /// # Arguments
    /// * `data` - The 16-bit value to set for the timer counters.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the value was successfully set, or an error if the write fails.
    pub fn set_timer_counter(&mut self, data: u16) -> Result<(), E> {
        self.write_register(RegisterTable::CompatibleTimerCounter0, (data & 0x00FF) as u8)?;
        self.write_register(RegisterTable::CompatibleTimerCounter1, ((data & 0xFF00) >> 8) as u8)
    }
}

impl<I2C, E> Rx8900<I2C>
where
    I2C: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
{
    /// Initializes the RTC with default settings.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if initialization was successful, or an error if the operation fails.
    pub fn init(&mut self) -> Result<(), E> {
        self.set_te(false)?;
        self.set_fsel0(false)?;
        self.set_fsel1(false)?;
        self.set_test(false)?;
        self.set_vdet()?;
        self.set_vlf()?;
        self.set_aie(false)?;
        self.set_tie(false)?;
        self.set_uie(false)?;

        // set VDETOFF=”1”
        self.set_voltage_detector_off(false)?;
        // set SWOFF=”1”
        self.set_switch_off(true)?;
        Ok(())
    }

    /// Reads the current date and time from the RTC.
    ///
    /// # Returns
    /// * `Result<NaiveDateTime, E>` - The current date and time, or an error if the read fails.
    pub fn datetime(&mut self) -> Result<NaiveDateTime, E> {
        let sec = self.sec()?;
        let min = self.min()?;
        let hour = self.hour()?;
        let day = self.day()?;
        let month = self.month()?;
        let yy = self.year()?;
        let date = NaiveDate::from_ymd_opt(2000 + yy as i32, month as u32, day as u32).unwrap();

        let mut time = NaiveTime::from_hms_opt(hour as u32, min as u32, sec as u32);
        if time.is_none() {
            time = NaiveTime::from_hms_opt(0, 0, 0);
        }
        Ok(NaiveDateTime::new(
            date,
            time.unwrap(),
        ))
    }

    /// Sets a full date and time in the RTC.
    ///
    /// # Arguments
    /// * `data` - The `NaiveDateTime` containing the date and time to be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the date and time were successfully set, or an error if the write fails.
    pub fn set_datetime(&mut self, data: NaiveDateTime) -> Result<(), E> {
        let date = data.date();
        let time = data.time();
        self.set_year((date.year() % 100) as u8)?;
        self.set_month(date.month() as u8)?;
        self.set_day(date.day() as u8)?;
        self.set_week(date.weekday())?;
        self.set_hour(time.hour() as u8)?;
        self.set_min(time.minute() as u8)?;
        self.set_sec(time.second() as u8)?;
        Ok(())
    }

    /// Sets or clears a specific bit in a register.
    ///
    /// # Arguments
    /// * `register` - The register where the bit will be modified.
    /// * `bit` - The bit position to set or clear (0-7).
    /// * `data` - Boolean value to set (`true`) or clear (`false`) the bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the bit was successfully modified, or an error if the operation fails.
    fn set_bit(&mut self, register: RegisterTable, bit: u8, data: bool) -> Result<(), E> {
        let current = self.read_register(register)?;
        let data = current & !(1 << bit) | (data as u8) << bit;
        self.write_register(register, data)
    }

    /// Sets the alarm type to either day or week alarm.
    ///
    /// # Arguments
    /// * `data` - The desired alarm type.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm type was successfully set, or an error if the operation fails.
    pub fn set_alarm_type(&mut self, data: AlarmType) -> Result<(), E> {
        self.set_wada(data == AlarmType::DayAlarm)
    }

    /// Sets the test bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the test bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the test bit was successfully set, or an error if the operation fails.
    pub fn set_test(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 7, data)
    }

    /// Sets the week alarm bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the week alarm bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the week alarm bit was successfully set, or an error if the operation fails.
    pub fn set_wada(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 6, data)
    }

    /// Sets the update interrupt type to either every minute or every second.
    ///
    /// # Arguments
    /// * `data` - The desired update interrupt type.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the update interrupt type was successfully set, or an error if the operation fails.
    pub fn set_update_interrupt_type(&mut self, data: UpdateInterruptType) -> Result<(), E> {
        self.set_usel(data == UpdateInterruptType::EveryMinute)
    }

    /// Sets the update interrupt selection bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the update interrupt selection bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the update interrupt selection bit was successfully set, or an error if the operation fails.
    pub fn set_usel(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 5, data)
    }

    /// Sets or clears the timer enable bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - True to enable the timer, false to disable it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer enable bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_te(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 4, data)
    }

    /// Sets the timer enable bit in the extension register to enable the timer.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer enable bit was successfully set, or an error if the operation fails.
    pub fn set_timer_enable(&mut self) -> Result<(), E> {
        self.set_te(true)
    }

    /// Resets the timer enable bit in the extension register to disable the timer.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer enable bit was successfully reset, or an error if the operation fails.
    pub fn reset_timer_enable(&mut self) -> Result<(), E> {
        self.set_te(false)
    }

    /// Sets the fsel1 bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the fsel1 bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the fsel1 bit was successfully set, or an error if the operation fails.
    pub fn set_fsel1(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 3, data)
    }

    /// Sets the fsel0 bit in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the fsel0 bit.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the fsel0 bit was successfully set, or an error if the operation fails.
    pub fn set_fsel0(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 2, data)
    }

    /// Configures the fout frequency selection bits based on the provided data.
    ///
    /// # Arguments
    /// * `data` - The fout frequency configuration as a byte where the relevant bits will be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the fout frequency was successfully configured, or an error if the operation fails.
    pub fn set_fsel(&mut self, data: u8) -> Result<(), E> {
        let current = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        let data = current & 0b11110011 | (data as u8) << 2;
        self.write_register(RegisterTable::CompatibleExtensionRegister, data)
    }

    /// Sets the frequency output (fout) control.
    ///
    /// # Arguments
    /// * `frequency` - The fout frequency setting from the `FoutFrequency` enum.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the fout frequency was successfully set, or an error if the operation fails.
    pub fn set_fout_frequency(&mut self, data: FoutFrequency) -> Result<(), E> {
        self.set_fsel(data as u8)
    }

    /// Enables or disables the specified source clock.
    ///
    /// # Arguments
    /// * `source_clock` - The source clock setting from the `SourceClock` enum.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the source clock was successfully set, or an error if the operation fails.
    pub fn set_source_clock(&mut self, data: SourceClock) -> Result<(), E> {
        let current = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        let data = current & 0b11111100 | (data as u8) << 0;
        self.write_register(RegisterTable::CompatibleExtensionRegister, data)
    }

    /// Sets the timer selection bit 1 in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the timer selection bit 1.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer selection bit 1 was successfully set, or an error if the operation fails.
    pub fn set_tsel1(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 1, data)
    }

    /// Sets the timer selection bit 0 in the extension register.
    ///
    /// # Arguments
    /// * `data` - The value to set for the timer selection bit 0.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer selection bit 0 was successfully set, or an error if the operation fails.
    pub fn set_tsel0(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleExtensionRegister, 0, data)
    }

    /// Configures the overall timer selection based on the provided bits.
    ///
    /// # Arguments
    /// * `data` - The timer selection configuration as a byte where the relevant bits will be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer selection was successfully configured, or an error if the operation fails.
    pub fn set_tsel(&mut self, data: u8) -> Result<(), E> {
        let current = self.read_register(RegisterTable::CompatibleExtensionRegister)?;
        let data = current & 0b11111100 | (data as u8) << 0;
        self.write_register(RegisterTable::CompatibleExtensionRegister, data)
    }

    /// Resets the update flag by setting the specific bit to false.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the update flag was successfully reset, or an error if the operation fails.
    pub fn set_uf(&mut self) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleFlagRegister, 5, false)
    }

    /// Alias for `set_uf`.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the update flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_update_flag(&mut self) -> Result<(), E> {
        self.set_uf()
    }

     /// Sets or clears the timer flag.
    ///
    /// # Arguments
    /// * `data` - True to set the timer flag, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_tf(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleFlagRegister, 4, data)
    }

    /// Resets the timer flag.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer flag was successfully reset, or an error if the operation fails.
    pub fn reset_timer_flag(&mut self) -> Result<(), E> {
        self.set_tf(false)
    }

    /// Sets or clears the alarm flag in the flag register.
    ///
    /// # Arguments
    /// * `data` - True to set the alarm flag, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_af(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleFlagRegister, 3, data)
    }

    /// Resets the alarm flag.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm flag was successfully reset, or an error if the operation fails.
    pub fn reset_alarm_flag(&mut self) -> Result<(), E> {
        self.set_af(false)
    }

    /// Sets or clears the voltage low flag in the flag register.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage low flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_vlf(&mut self) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleFlagRegister, 1, false)
    }

    /// Alias for `set_vlf`.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage low flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_voltage_low_flag(&mut self) -> Result<(), E> {
        self.set_vlf()
    }

    /// Sets or clears the voltage detect flag in the flag register.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage detect flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_vdet(&mut self) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleFlagRegister, 0, false)
    }

    /// Alias for `set_vdet`.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage detect flag was successfully set or cleared, or an error if the operation fails.
    pub fn set_voltage_detect_flag(&mut self) -> Result<(), E> {
        self.set_vdet()
    }

    /// Sets or clears the first compensation interval selection bit in the control register.
    ///
    /// # Arguments
    /// * `data` - True to set the bit, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the compensation interval bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_csel1(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 7, data)
    }

    /// Sets or clears the second compensation interval selection bit in the control register.
    ///
    /// # Arguments
    /// * `data` - True to set the bit, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the compensation interval bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_csel0(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 6, data)
    }

    /// Configures the compensation interval type in the control register.
    ///
    /// # Arguments
    /// * `data` - The compensation interval configuration as a byte where the relevant bits will be set.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the compensation interval was successfully configured, or an error if the operation fails.
    pub fn set_csel(&mut self, data: u8) -> Result<(), E> {
        let current = self.read_register(RegisterTable::CompatibleControlRegister)?;
        let data = (current & 0b00111111) | (data << 6);
        self.write_register(RegisterTable::CompatibleControlRegister, data)
    }

    /// Enables or disables the compensation interval type setting.
    ///
    /// # Arguments
    /// * `data` - The compensation interval from the `CompensationIntervalType` enum.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the compensation interval was successfully set, or an error if the operation fails.
    pub fn set_compensation_interval_type(&mut self, data: CompensationIntervalType) -> Result<(), E> {
        self.set_csel(data as u8)
    }

    /// Sets or clears the update interrupt enable bit.
    ///
    /// # Arguments
    /// * `data` - True to enable the update interrupt, false to disable it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the update interrupt enable bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_uie(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 5, data)
    }

    /// Enables or disables update interrupts.
    ///
    /// # Arguments
    /// * `enable` - Set to `true` to enable update interrupts, or `false` to disable them.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the setting was successfully applied, or an error if the operation fails.
    pub fn set_update_interrupt_enable(&mut self, data: bool) -> Result<(), E> {
        self.set_uie(data)
    }

    /// Sets or clears the timer interrupt enable bit.
    ///
    /// # Arguments
    /// * `data` - True to enable the timer interrupt, false to disable it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the timer interrupt enable bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_tie(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 4, data)
    }

    /// Enables timer interrupts.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the setting was successfully applied, or an error if the operation fails.
    pub fn set_timer_interrupt_enable(&mut self) -> Result<(), E> {
        self.set_tie(true)
    }

    /// Disables timer interrupts.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the setting was successfully applied, or an error if the operation fails.
    pub fn reset_timer_interrupt_enable(&mut self) -> Result<(), E> {
        self.set_tie(false)
    }

    /// Sets or clears the alarm interrupt enable bit.
    ///
    /// # Arguments
    /// * `data` - True to enable the alarm interrupt, false to disable it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the alarm interrupt enable bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_aie(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 3, data)
    }

    /// Enables alarm interrupts.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the setting was successfully applied, or an error if the operation fails.
    pub fn set_alarm_interrupt_enable(&mut self) -> Result<(), E> {
        self.set_aie(true)
    }

    /// Disables alarm interrupts.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the setting was successfully applied, or an error if the operation fails.
    pub fn reset_alarm_interrupt_enable(&mut self) -> Result<(), E> {
        self.set_aie(false)
    }

    /// Sets or clears the reset bit.
    ///
    /// # Arguments
    /// * `data` - True to initiate a reset, false otherwise.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the reset bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_reset(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::CompatibleControlRegister, 0, data)
    }

    /// Sets or clears the voltage detector off bit.
    ///
    /// # Arguments
    /// * `data` - True to turn off the voltage detector, false to keep it on.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage detector off bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_vdetoff(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::ExtendedBackupFunction, 3, data)
    }

    /// Alias for `set_vdetoff`.
    ///
    /// # Arguments
    /// * `data` - True to turn off the voltage detector, false to keep it on.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the voltage detector off bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_voltage_detector_off(&mut self, data: bool) -> Result<(), E> {
        self.set_vdetoff(data)
    }

    /// Sets or clears the switch off bit.
    ///
    /// # Arguments
    /// * `data` - True to turn off the switch, false to keep it on.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the switch off bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_swoff(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::ExtendedBackupFunction, 2, data)
    }

    /// Alias for `set_swoff`.
    ///
    /// # Arguments
    /// * `data` - True to turn off the switch, false to keep it on.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the switch off bit was successfully set or cleared, or an error if the operation fails.
    pub fn set_switch_off(&mut self, data: bool) -> Result<(), E> {
        self.set_swoff(data)
    }

    /// Sets or clears the backup mode sample bit 1.
    ///
    /// # Arguments
    /// * `data` - True to set the backup mode sample bit 1, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the backup mode sample bit 1 was successfully set or cleared, or an error if the operation fails.
    pub fn set_bksmp1(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::ExtendedBackupFunction, 1, data)
    }

    /// Sets or clears the backup mode sample bit 0.
    ///
    /// # Arguments
    /// * `data` - True to set the backup mode sample bit 0, false to clear it.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the backup mode sample bit 0 was successfully set or cleared, or an error if the operation fails.
    pub fn set_bksmp0(&mut self, data: bool) -> Result<(), E> {
        self.set_bit(RegisterTable::ExtendedBackupFunction, 0, data)
    }

    /// Configures the backup mode sampling time by setting the corresponding bits.
    ///
    /// # Arguments
    /// * `data` - The value to set for the backup mode sampling time.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the backup mode sampling time was successfully configured, or an error if the operation fails.
    pub fn set_bksmp(&mut self, data: u8) -> Result<(), E> {
        let current = self.read_register(RegisterTable::ExtendedBackupFunction)?;
        let data = (current & 0b11111100) | (data & 0b00000011);
        self.write_register(RegisterTable::ExtendedBackupFunction, data)
    }
    
    /// Alias for `set_bksmp`.
    ///
    /// # Arguments
    /// * `data` - The value to set for the backup mode sampling time.
    ///
    /// # Returns
    /// * `Result<(), E>` - Ok if the backup mode sampling time was successfully configured, or an error if the operation fails.
    pub fn set_backup_mode_sampling_time(&mut self, data: u8) -> Result<(), E> {
        self.set_bksmp(data)
    }
}
