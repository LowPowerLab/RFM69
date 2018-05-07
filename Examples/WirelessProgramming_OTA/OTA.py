#**********************************************************************************
# This script will handle the transmission of a compiled sketch in the
# form of an INTEL HEX flash image to an attached gateway/master Moteino node,
# for further wireless transmission to a target Moteino node that will receive it de-HEXified and
# store it in external memory. Once received by the target (which is also loaded with a custom bootloader
# capable of reading back that image) it will reset and reprogram itself with the new sketch
#
# EXAMPLE command line: python WirelessProgramming.py -f PathToFile.hex -s COM100 -t 123
# where -t is the target ID of the Moteino you are programming
# and -s is the serial port of the programmer Moteino (on linux/osx it is something like ttyAMA0)
# To get the .hex file path go to Arduino>file>preferences and check the verbosity for compilation
#   then you will get the path in the debug status area once the sketch compiles
#**********************************************************************************
# Copyright Felix Rusu, LowPowerLab.com
# Library and code by Felix Rusu - lowpowerlab.com/contact
#**********************************************************************************
# License
#**********************************************************************************
# This program is free software; you can redistribute it 
# and/or modify it under the terms of the GNU General    
# Public License as published by the Free Software       
# Foundation; either version 3 of the License, or        
# (at your option) any later version.                    
#                                                        
# This program is distributed in the hope that it will   
# be useful, but WITHOUT ANY WARRANTY; without even the  
# implied warranty of MERCHANTABILITY or FITNESS FOR A   
# PARTICULAR PURPOSE. See the GNU General Public        
# License for more details.                              
#                                                        
# You should have received a copy of the GNU General    
# Public License along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#                                                        
# Licence can be viewed at                               
# http://www.gnu.org/licenses/gpl-3.0.txt
#
# Please maintain this license information along with authorship
# and copyright notices in any redistribution of this code
# **********************************************************************************
import time, sys, serial
import collections
import re
from sys import exit

### GENERAL SETTINGS ###
SERIALPORT = "COM100"  # the default com/serial port the receiver is connected to
BAUDRATE = 115200      # default baud rate we talk to Moteino
TARGET=0               # Node ID of the Target that is being OTA reflashed
HEX = "flash.hex"      # the HEX file containing the new program for the Target
LINESPERPACKET = 3     # HEX lines to send per RF packet (1,2 or 3)
retries = 2
DEBUG = False

# Read command line arguments
if (sys.argv and len(sys.argv) > 1):
  if len(sys.argv)==2 and (sys.argv[1] == "-h" or sys.argv[1] == "-help" or sys.argv[1] == "?"):
    #print " -d or -debug         Turn debugging ON (verbose output)"
    print " -f or -file          HEX file to upload (Default: ", HEX, ")"
    print " -t or -target {ID}   Specify WirelessProgramming node target"
    print " -l or -lines {1,2,3} HEX lines per RF packet (Default: 3)"
    print " -s or -serial {port} Specify serial port of WirelessProgramming gateway (Default: ", SERIALPORT, ")"
    print " -b or -baud {baud}   Specify serial port baud rate (Default: ", BAUDRATE, ")"
    print " -h or -help or ?     Print this message"
    exit(0)
    
  for i in range(len(sys.argv)):
  #{
    #if sys.argv[i] == "-d" or sys.argv[i] == "-debug":
    #  DEBUG = True
    if (sys.argv[i] == "-s" or sys.argv[i] == "-serial") and len(sys.argv) >= i+2:
      SERIALPORT = sys.argv[i+1]
    if (sys.argv[i] == "-b" or sys.argv[i] == "-baud") and len(sys.argv) >= i+2:
      BAUD = sys.argv[i+1]
    if (sys.argv[i] == "-f" or sys.argv[i] == "-file") and len(sys.argv) >= i+2:
      HEX = sys.argv[i+1].strip()
    if (sys.argv[i] == "-t" or sys.argv[i] == "-target") and len(sys.argv) >= i+2:
      if sys.argv[i+1].isdigit() and int(sys.argv[i+1])>0 and int(sys.argv[i+1])<=255:
        TARGET = int(sys.argv[i+1])
      else:
        print "TARGET invalid  (", sys.argv[i+1], "), must be 1-255."
        exit(1)
    if (sys.argv[i] == "-l" or sys.argv[i] == "-lines") and len(sys.argv) >= i+2:
      if sys.argv[i+1].isdigit() and int(sys.argv[i+1])>0 and int(sys.argv[i+1])<=3:
        LINESPERPACKET = int(sys.argv[i+1])
      else:
        print "LINESPERPACKET invalid  (", sys.argv[i+1], "), must be 1-3."
        exit(1)
  #}

def LOG(message):
  sys.stdout.write(message)

def LOGln(message):
  sys.stdout.write('\n' + message)
  
def millis():
  return int(round(time.time() * 1000))
  
def serWriteln(ser, msg):
  #ser.write(msg + '\n')
  ser.write(bytes((msg+'\n').encode('utf-8')))

HANDSHAKE_OK            = 0
HANDSHAKE_FAIL          = 1
HANDSHAKE_FAIL_TIMEOUT  = 2
HANDSHAKE_ERROR         = 3

def waitForHandshake(isEOF=False):
  now = millis()
  while True:
    if millis()-now < 4000:
    #{
      if isEOF:
        serWriteln(ser, "FLX?EOF")
      else:
        serWriteln(ser, "FLX?")
        LOGln("FLX?")
      ser.flush()
      rx = ser.readline().rstrip().upper()

      if len(rx) > 0:
      #{
        LOGln("Moteino: [" + rx + "]")
        if rx == "FLX?OK":
          LOGln("HANDSHAKE OK!")
          return HANDSHAKE_OK
        elif rx == "FLX?NOK":
          LOGln("HANDSHAKE FAIL [TIMEOUT]: " + rx)
          return HANDSHAKE_FAIL
        elif (len(rx)>7 and rx.startswith("FLX?NOK") or rx.startswith("FLX?ERR")):
          LOGln("HANDSHAKE FAIL [HEX IMG refused by target node], reason: " + rx)
          return HANDSHAKE_FAIL_ERROR
      #}
    #}
    else: return HANDSHAKE_FAIL_TIMEOUT

def waitForTargetSet(targetNode):
  now = millis()
  to = "TO:" + str(TARGET)
  LOGln(to)
  serWriteln(ser, to)
  ser.flush()
  while True:
  #{
    if millis()-now < 3000:
    #{
      rx = ser.readline().rstrip()
      if len(rx) > 0:
      #{
        LOGln("Moteino: [" + rx + "]")
        if rx == to + ":OK":
          return True
        else: return False
      #}
    #}
  #}
  return False

# return 0:timeout, 1:OK!, 2:match but out of synch
def waitForSEQ(seq):
  now = millis()
  while True:
    if millis()-now < 3000:
      rx = ser.readline().strip()

      if (rx.upper().startswith("RFTX >") or rx.upper().startswith("RFACK >")):
      #{
          LOGln("Moteino DEBUG: " + rx)
          rx = ""
          continue
      #}

      if len(rx) > 0:
      #{
        pattern = "FLX:([0-9]*):OK"
        LOG(" RX > " + rx)
        result = re.match(pattern, rx)
        if result != None:
          if int(result.group(1)) == seq:
            return 1
          else: return 2
        else: LOGln("Programmer reply '"+rx+"' does not match expected pattern: '"+pattern+"'")
      #}
    else: return 0

# MAIN()
#if __name__ == "__main__":
try:
  start = millis();
  # open up the serial port to get data transmitted to Programmer Moteino
  ser = serial.Serial(SERIALPORT, BAUDRATE, timeout=1) #timeout=0 means nonblocking
  ser.setDTR(False)
  ser.setRTS(False)
  time.sleep(2) #wait for Programmer Moteino reset after port open and potential bootloader time (~1.6s) 
  ser.flushInput();
except IOError as e:
  LOGln("COM Port [", SERIALPORT, "] not found, exiting...")
  exitNow(1)

try:
  if not 0<TARGET<= 255:
    LOGln("TARGET not provided (use -h for help), now exiting...")
    exit(1)
  
  #send target ID first
  if waitForTargetSet(TARGET):
    LOGln("TARGET SET OK")
  else:
    LOGln("TARGET SET FAIL, exiting...")
    exit(1)
  
  with open(HEX) as f:
    LOGln("File found, passing to Moteino OTA Programmer...")
    
    handshakeResponse = waitForHandshake()
    if (handshakeResponse == HANDSHAKE_OK):
      seq = 0
      packetCounter = 0
      content = f.readlines()

      while seq < len(content):
      #{
        currentLine = content[seq].strip()
        isEOF = (content[seq].strip() == ":00000001FF") #this should be the last line in any valid intel HEX file
        result = -1
        bundledLines = 1
        
        if isEOF==False:
        #{
          hexDataToSend = currentLine

          #bundle 2 or 3 HEX lines in 1 RF packet (assuming: 1 HEX line has 16 bytes of data, following HEX lines also each being 16 bytes)
          if LINESPERPACKET > 1 and currentLine[7:9] == '00':
          #{
            #check if next line != EOF, so we can bundle 2 lines
            nextLine = content[seq+1].strip()
            if nextLine != ":00000001FF" and nextLine[7:9] == '00':
            #{
              #need to sum: the 2 lines checksums + address bytes of nextLine (to arrive at a correct final checksum of combined 2 lines
              checksum = int(currentLine[len(currentLine)-2:len(currentLine)], 16) + int(nextLine[len(nextLine)-2:len(nextLine)], 16) + int(nextLine[3:5], 16) + int(nextLine[5:7], 16)
              addressByte = int(currentLine[1:3], 16) + int(nextLine[1:3], 16)
              
              #check if a third line != EOF, so we can bundle 3 lines
              nextLine2 = content[seq+2].strip()
              if LINESPERPACKET==3 and nextLine2 != ":00000001FF" and nextLine2[7:9] == "00":
              #{
                #need to sum: the previous checksum + address bytes of nextLine2 (to arrive at a correct final checksum of combined 3 lines
                checksum += int(nextLine2[len(nextLine2)-2:len(nextLine2)], 16) + int(nextLine2[3:5], 16) + int(nextLine2[5:7], 16)
                addressByte += int(nextLine2[1:3], 16)
                hexDataToSend = ":" + ('%0*X' % (2,addressByte%256)) + hexDataToSend[3:len(hexDataToSend)-2] + nextLine[9:len(nextLine)-2] + nextLine2[9:len(nextLine)-2] + ('%0*X' % (2,checksum%256))
                bundledLines=3
              #}
              else:
              #{
                hexDataToSend = ":" + ('%0*X' % (2,addressByte%256)) + hexDataToSend[3:len(hexDataToSend)-2] + nextLine[9:len(nextLine)-2] + ('%0*X' % (2,checksum%256))
                bundledLines=2;
              #}
            #}
          #}
          tx = "FLX:" + str(packetCounter) + hexDataToSend
          LOGln("TX > " + tx)
          serWriteln(ser, tx)
          result = waitForSEQ(packetCounter)
        #}
        elif waitForHandshake(True) == HANDSHAKE_OK:
        #{
          LOGln("SUCCESS! (time elapsed: " + ("%.2f" % ((millis()-start)/1000.0)) + "s)")
          exit(0);
        #}
        else:
        #{
          LOGln("FAIL, IMG REFUSED BY TARGET (size exceeded? verify target MCU matches compiled target)")
          exit(99)
        #}

        if result == 1:
        #{
          seq+=bundledLines
          packetCounter+=1
        #}
        elif result == 2: # out of synch, retry
        #{
          if retries > 0:
            retries-=1
            LOGln("OUT OF SYNC: retrying...\n")
            continue
          else:
            LOGln("FAIL: out of sync (are you running the latest OTA libs/sources?)")
            exit(1)
        #}
        else:
        #{
          if retries > 0:
            retries-=1
            LOGln("Timeout, retry...\n")
            continue
          else:
          #{
            LOGln("FAIL: timeout (are you running the latest OTA libs/sources?)")
            exit(1)
          #}
        #}
      #}

      while 1:
      #{
        rx = ser.readline()
        if (len(rx) > 0): LOGln(rx.strip())
      #}

    elif (handshakeResponse == HANDSHAKE_FAIL_TIMEOUT):
      LOGln("FAIL: No response from Moteino programmer, is it connected to " + port)
      exit(1)
    elif (handshakeResponse == HANDSHAKE_FAIL_TIMEOUT):
      LOGln("FAIL: No response from Moteino programmer, is it connected to " + port)
      exit(1)
    else:
      LOGln("FAIL: No response from Moteino Target, is Target listening on same Freq/NetworkID & OTA enabled?")
      exit(1)

except IOError:
  LOGln("File [", HEX, "] not found, exiting...")
  exit(1)
  
finally:
  #print 'FINALLY' + '\n'
  ser.close()