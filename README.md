RFM69 Library
----------------
By Felix Rusu (felix@lowpowerlab.com)
<br/>
RFM69 library for RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)

##License
GPL 3.0, please see the License.txt file


##Features
Among others, this is a set of features implemented in this library:

- easy to use API with a few simple functions for basic usage
- 255 possible nodes on 256 possible networks
- 61 bytes max message length (limited to 61 to support AES hardware encryption)
- customizable transmit power (32 levels) for low-power transmission control
- sleep function for power saving
- automatic ACKs with the sendWithRetry() function
- hardware 128bit AES encryption
- hardware preamble, synch recognition and CRC check
- digital RSSI can be read at any time with readRSSI()
- interrupt driven
- tested on [Moteino R3, R4, R4-USB (ATMega328p)](http://lowpowerlab.com/shop/Moteino-R4)
- works with RFM69W, RFM69HW, RFM69CW, RFM69HCW, Semtech SX1231/SX1231H transceivers
- promiscuous mode allows any node to listen to any packet on same network

I consider this an initial beta release, it could contain bugs, but the provided Gateway and Node examples should work out of the box. Please let me know if you find issues.

###Installation
Copy the content of this library in the "Arduino/libraries/RFM69" folder.
<br />
To find your Arduino folder go to File>Preferences in the Arduino IDE.
<br/>
See [this tutorial](http://learn.adafruit.com/arduino-tips-tricks-and-techniques/arduino-libraries) on Arduino libraries.

###MISC / possible issues
- The library and examples are continuously improved as bugs and stability issues are discovered. Be sure to check back often for changes.
- Moteino boards are loaded with fuses that will delay startup. This means that other boards like Duemilanove/UNO might need a delay() in the setup() function before doing anything - to allow the transceiver to power up.

###Sample usage
- [Node](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Node/Node.ino)
- [Gateway](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Gateway/Gateway.ino)

##Blog writeup
http://lowpowerlab.com/blog/2013/06/20/rfm69-library/

##Why
- I have spent a lot of time developing this library for RFM69W/HW transceivers. I made it open source because I believe a lot of people can benefit from this new powerful transceiver. I hope people will also contribute and build on my work
- I have long researched alternative transceivers for RFM12B which is still an excellent transceiver but it is much lower output power and has limited built in features which need to be implemented in firmware (PREAMBLE, SYNC, CRC, packet engine, encryption etc).
- I wanted a transceiver that could still be very small, easy to use, but have the longer range that I wanted
- RFM69 comes in 2 variants that have the same layout/connections: RFM69W (13dBm, 45mA TX) and RFM69HW (20dBm, 130mA TX)

##RFM69W range
- I have tested open-air range on these transceivers (the W only) in various combinations.
- I am happy to say that a range of upwards of 350m can be achieved. I went to local parks and in very large parking spaces and I ran out of space, so more than 350m is possible. Some users reported upwards of 500m by lowering the bitrate, and a forum user reported 1.5miles at 1.2Kbps: see http://lowpowerlab.com/forum/index.php/topic,112.msg288.html and http://lowpowerlab.com/moteino/#antennas
- The caveat with these higher RF power units is that they need more DC power when they transmit. For battery powered motes, you will need to keep them powered down and only transmit periodically. Use the sleep() function to put the radios in low power mode and use the [LowPower](https://github.com/rocketscream/Low-Power) or [Narcoleptic](https://code.google.com/p/narcoleptic/) libraries to power down your arduino

##License
GPL 3.0. See License.txt file.
