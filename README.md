# SlimLoRa library

This repository contains the SlimLoRa LoRaWAN library with a sample project for ATtiny85 chips. It relies on the AES implementation by Ideentron. It is compatible with the HopeRF RFM95 and the Semtech SX1276 radio chips.

Currently this library is heavily optimized for ATtiny85 chips and needs changes to run on other devices (e.g. Arduino). Furthermore, currently only the EU band around 868 MHz is supported.

SlimLoRa implements the ABP and OTAA activation schemes. It has support for downlink messages and the Adaptive Data Rate mechanism.

This library *is not* fully LoRaWAN 1.0 compliant. Please verify its behavior before using it on public networks.
It also *does not* enforce a duty cycle. This must be ensured by the user.

**Contents:**

- [Installing](#installing)
- [Features](#features)
- [Requirements and Limitations](#requirements-and-limitations)
- [Configuration](#configuration)
- [Supported hardware](#supported-hardware)
- [Example Program](#example-sketches)
- [Timing](#timing)
- [Release History](#release-history)
- [Trademark Acknowledgements](#trademark-acknowledgements)
- [License](#license)

## Installing

To compile and flash this library, attach your USB-ASP programmer and execute the following commands:

```bash
git clone https://github.com/novag/LoRa-ATtiny-Node
cd LoRa-ATtiny-Node
cp config.h.example config.h # Insert your key(s)
make
make program
```

## Features

This library provides a usable LoRaWAN 1.0 Class A implementation supporting the EU-868 band.

The library has some support for LoRaWAN 1.1 which is not usable yet.

- Sending uplink packets, not taking into account duty cycles.
- Message encryption and message integrity checking.
- Over-the-Air Activation (OTAA).
- Activation by Personalization (ABP).
- Adaptive Data Rate for spreading factors (ADR).
- Receiving downlink packets in the RX1 and RX2 windows.
- MAC command processing for LinkAdr and RxParamSetup requests.

## Requirements and Limitations

### No event handling
This library does not implement any kind of event handling. A call to the Join and Transmit methods will block for several seconds until the end of the second receive window.

### Timer0
SlimLoRa uses the Timer0 on every call of the Join or Transmit methods. Between these calls Timer0 can be used by the user for other purposes.
See also timing.h and timing.c.

### Memory requirements
SlimLoRa uses about 348 bytes of SRAM for its stack and about 64 bytes for global symbols.

The current project has a maximum total SRAM usage of 460 bytes.
On an ATtiny85 you should never call into the Join and Transmit methods from deeply nested functions. Analyze and profile your program!

## Configuration

Attached sensors can be configured in the config.h header file. You should also configure the DeviceEUI, JoinEUI (AppEUI) and the keys in this file.

Enabling and disabling the ADR mechanism can be done using the SetAdrEnabled method.

#### Debug output

Disable all sensors and attach a UART console to pin 4.

```bash
make DEBUG=1
make program
```

## Supported hardware

ATtiny85 with radios based on the SX1276 are supported (e.g. HopeRF RFM95).

## Example Program

This repository contains a working sample project that sends temperature, humidity and voltage data every 15 minutes to the network.
The user can choose between ABP and OTAA activation and can enable or disable ADR.

The ATtiny can act as an SPI, I2C and Dallas 1-Wire master. Thus the following three sensors are currently supported:
- DHT22 (Non-standardized OneWire)
- SI7021 (I2C slave)
- DS18B20 (Dallas 1-Wire)

This means, that with the available ports, in addition to the radio module, either one non-standardized OneWire device, multiple I2C devices, multiple Dallas 1-Wire devices or one additonal SPI slave device can be connected.

Only select a single device in the configuration file. Otherwhise the first selected device will take precedence.

## Timing

*TODO*

## Release History

- V0.1.0 Initial release.

## Contributions

This library started from parts written by Ideetron B.V.. Of these, almost only the AES routines are left.

- Hendrik Hagendorn - OTAA/downlink/timing implementation, ATtiny85 optimization, ...

- Ideentron B.V. - [RFM95W_Nexus](https://github.com/Ideetron/RFM95W_Nexus)

## Trademark Acknowledgements

LoRa is a registered trademark of Semtech Corporation. LoRaWAN is a registered trademark of the LoRa Alliance.

All other trademarks are the properties of their respective owners.

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see [<http://www.gnu.org/licenses/>](http://www.gnu.org/licenses/).
