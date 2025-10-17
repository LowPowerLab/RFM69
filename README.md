# RFM69 Library
[![arduino-library-badge](https://www.ardu-badge.com/badge/RFM69_LowPowerLab.svg?)](https://www.ardu-badge.com/RFM69_LowPowerLab)
[![Build Status](https://app.travis-ci.com/LowPowerLab/RFM69.svg)](https://app.travis-ci.com/LowPowerLab/RFM69)
[![GitHub release](https://img.shields.io/github/release/LowPowerLab/RFM69.svg)](https://github.com/LowPowerLab/RFM69)
[![GitHub issues](https://img.shields.io/github/issues/LowPowerLab/RFM69.svg)](https://github.com/LowPowerLab/RFM69/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/LowPowerLab/RFM69.svg)](https://github.com/LowPowerLab/RFM69/pulls)
[![license](https://img.shields.io/github/license/LowPowerLab/RFM69.svg)](https://github.com/LowPowerLab/RFM69/blob/master/LICENSE.txt)

By Felix Rusu, [LowPowerLab.com](http://LowPowerLab.com)
<br/>
RFM69 library for RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)
<br/>
The latest examples, new features and bug fixes are found in the [original repository](https://github.com/LowPowerLab/RFM69) of this library.

## License
The default license is GPL 3.0, please see the [License.txt](https://github.com/LowPowerLab/RFM69/blob/master/License.txt) file for details. Be sure to include the same license with any fork or redistribution of this library.
<br/>
You are free to use the library for personal or commercial purposes but you must adhere to its copyleft requirements. This means that any modified or derivative work you distribute must also be licensed under GPL, and you must provide the source code to recipients.
<br/>
**For any other custom licensing arrangements where these terms are not acceptable, you must contact the [author directly]([url](https://github.com/LowPowerLab)) or use the [contact form through LowPowerLab.com](lowpowerlab.com/contact).**

## Features
- easy to use API with a few simple functions for basic usage
- 1023 possible nodes on 256 possible networks
- 61 bytes max message length (limited to 61 to support AES hardware encryption)
- customizable transmit power (32 levels) for low-power transmission control
- sleep function for power saving
- automatic ACKs with the `sendWithRetry()` function
- hardware 128bit AES encryption
- hardware preamble, synch recognition and CRC check
- digital RSSI can be read at any time with `readRSSI()`
- interrupt driven
- tested on [all Moteinos & many custom boards designed by LowPowerLab](https://lowpowerlab.com/shop/moteino)
- works with RFM69W, RFM69HW, RFM69CW, RFM69HCW, Semtech SX1231/SX1231H transceivers
- promiscuous mode allows any node to listen to any packet on same network

### Library Installation
#### Arduino IDE
Copy the content of this library in the `Arduino/libraries/RFM69` folder.
<br />
To find your Arduino folder go to `File -> Preferences` in the Arduino IDE.
<br/>
See [this tutorial](https://www.arduino.cc/en/Guide/Libraries) on Arduino libraries.

#### PlatformIO
See [PlatformIO registry](https://registry.platformio.org/libraries/lowpowerlab/RFM69) for latest installation details.

### Hardware & programming
The easiest way to get started is with the well documented and supported [Moteino](http://moteino.com) microcontroller platform which is [easily programmable](https://lowpowerlab.com/programming) from the Arduino IDE. This includes the [Moteino, MoteinoUSB & MoteinoMEGA](https://lowpowerlab.com/shop/Moteino). RFM69 transceivers were extensively tested on Moteinos for the purpose of building internet of things (IoT) devices that can be controlled wirelessly. This platform has matured over time and there is now a [dedicated page](https://lowpowerlab.com/gateway) where you can review how these devices can interact with each other via a RaspberryPi gateway interface. Here's a video overview:<br/>
https://www.youtube.com/watch?v=YUUZ6k0pBHg
<br/>
https://www.youtube.com/watch?v=I9MNZQgqKHA
<br/>
https://www.youtube.com/watch?v=F15dEqZ4pMM

### Basic sample usage
- The [Gateway](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Gateway/Gateway.ino) example listens for incoming data from remote nodes and responds to any ACK requests
- The [Node](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Node/Node.ino) example is a loop that sends increasingly longer packets to the gateway and waits for an ACK each time
- More examples are added from time to time, check all the [examples](https://github.com/LowPowerLab/RFM69/tree/master/Examples), visit the [LowPowerLab blog](http://lowpowerlab.com) for latest news and projects, and check out the [LowPowerLab forums](https://lowpowerlab.com/forum) for projects and discussion

## Blog writeup
See the [library release blog post](http://lowpowerlab.com/blog/2013/06/20/rfm69-library/)

## Why RFM69
- I have spent a lot of time developing this library for RFM69W/CW/HW/HCW transceivers. I made it open source because I believe a lot of people can benefit from this new powerful transceiver. I hope others will also contribute and build on my work
- I have long researched alternative transceivers for RFM12B which is still an excellent transceiver but it is much lower output power and has limited built in features which need to be implemented in firmware (PREAMBLE, SYNC, CRC, packet engine, encryption etc).
- I wanted a transceiver that could still be very small, easy to use, and have the longer range that I needed
- RFM69 comes in 2 variants: RFM69W/CW (13dBm, 45mA TX) and RFM69HW/HCW (20dBm, 130mA TX). They are not pin compatible. The RFM69HCW is generally recommended since it has the extra power when needed.

## RFM69 range and antennas
- I have tested open-air range on these transceivers in various combinations.
- I am happy to say that a range of upwards of 400m can be achieved with the default settings provided in the library. Some users reported upwards of 500m by lowering the bitrate, and a forum user reported 1.5miles at 1.2Kbps: see [this forum post](http://lowpowerlab.com/forum/index.php/topic,112.msg288.html) and [this blog page](http://lowpowerlab.com/moteino/#antennas)
- The caveat with these higher RF power units is that they need more DC power when they transmit. For battery powered motes, you will need to keep them powered down and only transmit periodically. Use the sleep() function to put the radios in low power mode and use the [LowPower](https://github.com/lowpowerlab/lowpower) library to power down your Moteino.
- The RFM69 ATC feature enables keeping transmit power only at the required level to achieve the desired RSSI at the receiving end, see examples for usage
