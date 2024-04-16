#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use panic_probe as _;
use rp_pico as bsp;

// USB Serial support
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// LED related types
use embedded_hal::digital::v2::OutputPin;

// RTC related types
use chrono::{Datelike, NaiveDateTime, Timelike};
use rx8900::Rx8900;

// alloc
extern crate alloc;
use alloc::format;
use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the USB bus
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set the serial port
    let mut serial = SerialPort::new(&usb_bus);

    // Set a USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x5824, 0x27dd))
        .strings(&[StringDescriptors::new(LangID::EN).product("Serial port")])
        .expect("Failed to set strings")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio16.reconfigure();
    let scl_pin = pins.gpio17.reconfigure();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let mut rx8900 = Rx8900::new(i2c);
    rx8900.init().unwrap();
    delay.delay_ms(10);

    let datetime = NaiveDateTime::new(
        chrono::NaiveDate::from_ymd_opt(2001, 2, 3).unwrap(),
        chrono::NaiveTime::from_hms_opt(4, 5, 6).unwrap(),
    );
    rx8900.set_datetime(datetime).unwrap();

    let mut count = 0;
    loop {
        delay.delay_ms(5);
        let _ = usb_dev.poll(&mut [&mut serial]);
        count += 1;
        if count == 200 {
            let datetime = rx8900.datetime().unwrap();
            let date = datetime.date();
            let time = datetime.time();
            led_pin.set_state((time.second() % 2 == 0).into()).unwrap();

            let text = format!(
                "{:04}-{:02}-{:02} {:02}:{:02}:{:02}\r\n",
                date.year(),
                date.month(),
                date.day(),
                time.hour(),
                time.minute(),
                time.second()
            );
            let _ = serial.write(text.as_bytes());
            count = 0;
        }
    }
}

// End of file
