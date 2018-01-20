/*
 * AeroRFBase.h
 *
 *  Created on: Oct 28, 2017
 *      Author: Joel Blackthorne
 *
 *  AeroRFLib is a radio library designed for long-distance, high-frequency,
 *  distance measurement as part of the AeroTracker project.
 *
 *
 *  This module is based on the RFM69HW_HCW and RFM69HW_HW HopeRF transceivers
 *  operating at 915mhz and will only work with those specific modules.
 *
 *  Extends work Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
 */
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************


#ifndef AERORFBASE_H_
#define AERORFBASE_H_

#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <RFM69.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
//#define NETWORKID     106  //the same on all nodes that talk to each other
//
////Node IDS determine the type of device, so the ide must be in the correct range
////Nodes 1-30 = Sensors
////Nodes 31 + are tags
//#define NODEID        1    //unique for each node on same network
//#define NODEID_SENSOR_CUTT_OFF 30 //max number for a sensor node id

//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "Aer0RadioFreq915" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
//*********************************************************************************************
#define SERIAL_EN     //Must be enabled
#define SERIAL_BAUD   115200
//#define SERIAL_BAUD   9600
#define SER_WRITE(input){Serial.write(input);}
#define SER_WRITELN(input){Serial.println(input);}

//*********************************************************************************************
//Configure the distance ping
//*********************************************************************************************
#define ACK_TIME       30  // # of ms to wait for an ack packet

//ATC Notes: DO NOT Enable Automatic Transmission Control!
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL

//Pin for status LED
//#define STATUS_LED         9

#define DEBUG_EN //comment out to disable debugging

#define PROD_NAME "AeroTracker RF"

#ifdef DEBUG_EN
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#define BLINK_DELAY 100
#define GUID_SEP 0x2D

#define EEPROM_ADDR_START 0
#define CHECK_BYTE_INITIAL_FLASH 0x0A

//Custom Types
#define AY_GUID_SIZE 36
#define AY_VERSION_SIZE 3
#define AY_DATE_SIZE 8
#define AY_REG_KEY_SIZE 10

typedef uint8_t AeroRFGUID[AY_GUID_SIZE];
typedef uint8_t AeroRFVersion[AY_VERSION_SIZE]; //3 bytes in form <major><minor><dot>
typedef uint8_t chdate[AY_DATE_SIZE];
typedef uint8_t AeroRFRegKey[AY_REG_KEY_SIZE];

//Common EEPROM structure used by all AeroRF Objects
typedef struct AeroEEPROM{
	uint8_t check_byte; //security check byte
	uint8_t network_id; //network id for RF
	uint8_t node_id;    //node id for RF
	AeroRFGUID guid;    //custom GUID set once
	chdate created_on; //Date of firmware and GUID set
	AeroRFVersion fw_version; //version number
	chdate registerd_on; //registration date in YYYYMMDD
	AeroRFRegKey regisration_key; //key of last registration
} AeroEEPROM;

// #############################################
// EEPROM Indexes
#define IS_CHECK_BYTE (EEPROM_ADDR_START)
#define IS_NETWORK_ID (IS_CHECK_BYTE + 1)
#define IS_NODE_ID (IS_NETWORK_ID + 1)
#define IS_GUID (IS_NODE_ID + 1)
#define IE_GUID (IS_GUID + AY_GUID_SIZE)
#define IS_CREATED_ON (IE_GUID + 1)
#define IE_CREATED_ON (IS_CREATED_ON + AY_DATE_SIZE)
#define IS_VERSION (IE_CREATED_ON + 1)
#define IE_VERSION (IS_VERSION + AY_VERSION_SIZE)
#define IS_REG_ON (IE_VERSION + 1)
#define IE_REG_ON (IS_REG_ON + AY_DATE_SIZE)
#define IS_REG_KEY (IE_REG_ON + 1)
#define IE_REG_KEY (IS_REG_KEY + AY_REG_KEY_SIZE)
// #############################################

class AeroRFBase {
public:
	RFM69 radio;
	AeroRFBase();
	virtual ~AeroRFBase();
	uint8_t getNodeId();
	uint8_t getNetworkId();
	AeroRFGUID* get_guid();
	void run_cycle();
	bool initialize();
	void blink(uint8_t pin);
	void print_info();
	void set_firmware_info(char* created_on,
			char* version,
			uint8_t networkId,
			uint8_t nodeId);
	void init_chdate(uint8_t *val);
	void ascii_array_to_byte(char* ascii_lst, uint8_t* byte_lst, uint16_t lst_size);
	void byte_array_to_ascii(uint8_t* byte_lst, char* ascii_lst, uint16_t lst_size);
	void byte_array_copy(uint8_t* source_list, uint8_t* target_list, uint16_t lst_size);
private:
	uint8_t _check_byte;
	uint8_t _nodeId;
	uint8_t _networkId;
	AeroRFGUID _guid;
	chdate _created_on;
	AeroRFVersion _fw_version;
	chdate _registered_on;
	AeroRFRegKey _registration_key;
	bool _blink_on;
	bool read_all_eeprom(AeroEEPROM eeprom_val);
	void load_eeprom();
	bool read_eeprom_char(unsigned char *val, int addr, int len);
	void write_eeprom_char(unsigned char *val, int addr, int len);
	void create_guid(AeroRFGUID guid);
	void version_str_to_bytes(char* version_list, AeroRFVersion ver_bytes);
};

#endif /* AERORFBASE_H_ */
