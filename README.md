# [max17262](https://crates.io/crates/max17262)

A platform agnostic driver to interface with the MAX17262 LiPo Fuel Gauge via I2C

This driver was built using [`embedded-hal`] traits.

[`embedded-hal`]: https://docs.rs/embedded-hal/~0.1

## Documentation

 Read the detailed documentation [here](https://docs.rs/max17262/)

## What works

- [] Read Voltage
- [] Read State of Charge
- [] Read Charge Rate

## What is missing

- Configuration Option 2: Custom Short INI without OCV Table
- Configuration Option 3: Custom Full INI with OCV Table

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  [APACHE](http://www.apache.org/licenses/LICENSE-2.0))

- MIT license ([LICENSE-MIT](LICENSE-MIT) or [MIT](http://opensource.org/licenses/MIT))

at your option.

## Resources

[Data Sheet](https://datasheets.maximintegrated.com/en/ds/MAX17262.pdf)
[User Guide](https://pdfserv.maximintegrated.com/en/an/MAX1726x-ModelGauge-m5-EZ-user-guide.pdf)
[Software Implementation Guide](https://pdfserv.maximintegrated.com/en/an/MAX1726x-Software-Implementation-user-guide.pdf)
