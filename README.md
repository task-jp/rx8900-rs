# rx8900

A rust device driver for the Epson RX8900SA/CE I2C-Bus Interface Real Time Clock Module.

## Usage

```rust
use rx8900::RX8900;

let i2c = /* ..i2c provider */
let mut delay = /* ..delay provider */

// initialize the RX8900 using the primary I2C address 0x32
let mut bme280 = RX8900::new_primary(i2c);

// initialize the rtc
bme280.init().unwrap();

// some delay
delay.delay_ms(10);

// set initial date time
let datetime = NaiveDateTime::new(
    chrono::NaiveDate::from_ymd_opt(2001, 2, 3).unwrap(),
    chrono::NaiveTime::from_hms_opt(4, 5, 6).unwrap(),
);
rx8900.set_datetime(datetime).unwrap();

// read date time from RX8900
let datetime = rx8900.datetime().unwrap();

```

## License

Licensed under either of:

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.