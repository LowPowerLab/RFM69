RFM69 Library
----------------
By Felix Rusu (felix@lowpowerlab.com)
<br/>
Creative Commons Attrib Share-Alike License
http://creativecommons.org/licenses/by-sa/3.0/

RFM69 library for RFM69W and RFM69HW (semtech SX1231, SX1231H)


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
- tested on [Moteino R3s (ATMega328p)](http://lowpowerlab.com/shop/Moteino-R3)
- works with RFM69W and RFM69HW, Semtech SX1231/SX1231H transceivers
- promiscuous mode allows any node to listen to any packet on same network

I consider this an initial beta release, it could contain bugs, but the provided Gateway and Node examples should work out of the box. Please let me know if you find issues.

###Installation
Copy the content of this library in the "Arduino/libraries/RFM12B" folder.
<br />
To find your Arduino folder go to File>Preferences in the Arduino IDE.
<br/>
See [this tutorial](http://learn.adafruit.com/arduino-tips-tricks-and-techniques/arduino-libraries) on Arduino libraries.

###Saple usage
- [Node](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Node/Node.ino)
- [Gateway](https://github.com/LowPowerLab/RFM69/blob/master/Examples/Gateway/Gateway.ino)

##Blog writeup
http://lowpowerlab.com/blog/2013/06/20/rfm69-library/