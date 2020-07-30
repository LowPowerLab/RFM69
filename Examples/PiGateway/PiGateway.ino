// **********************************************************************************************************
// Moteino gateway/base sketch that works with Moteinos equipped with RFM69 transceiver
// It will buffer the serial data to ensure host serial requests are not missed.
// This is a buffered gateway sketch that receives packets from end node Moteinos, formats them as ASCII strings
//      with the end node [ID] and passes them to Pi/host computer via serial port
//     (ex: "messageFromNode" from node 123 gets passed to serial as "[123] messageFromNode")
// It also listens to serial messages that should be sent to listening end nodes
//     (ex: "123:messageToNode" sends "messageToNode" to node 123)
// Make sure to adjust the settings to match your transceiver settings (frequency, HW etc).
// **********************************************************************************
// Copyright Felix Rusu 2020, http://www.LowPowerLab.com/contact
// **********************************************************************************
#include <RFM69.h>      //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>  //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>  //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>   //get it here: https://github.com/lowpowerlab/spiflash
#include <PString.h>    //easy string manipulator: http://arduiniana.org/libraries/pstring/
#include <Streaming.h>  //easy C++ style output operators: http://arduiniana.org/libraries/streaming/
//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID          1  //the gateway has ID=1
#define NETWORKID     200  //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
//#define FREQUENCY_EXACT 916000000 //uncomment and set to a specific frequency in Hz, if commented the center frequency is used
#define ENCRYPTKEY    "sampleEncryptKey" //identical 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW  //required for RFM69HW/HCW, comment out for RFM69W/CW!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ENABLE_WIRELESS_PROGRAMMING
//*****************************************************************************************************************************
#define DEBUG_EN            //comment out if you don't want any serial verbose output
#define SERIAL_BAUD   115200 // Serial baud rate must match your Pi/host computer serial port baud rate!
//*****************************************************************************************************************************
#ifdef DEBUG_EN
  #define DEBUG(input)   Serial.print(input)
  #define DEBUGln(input) Serial.println(input)
#else
  #define DEBUG(input)
  #define DEBUGln(input)
#endif

#define PRINT_UPTIME Serial<< F("UPTIME:") << millis() << endl;
#define PRINT_FREQUENCY Serial << F("SYSFREQ:") << radio.getFrequency() << endl;

#define LED_HIGH digitalWrite(LED_BUILTIN, HIGH)
#define LED_LOW digitalWrite(LED_BUILTIN, LOW)
//******************************************** BEGIN ADVANCED variables ********************************************************************************
#if defined(MOTEINO_M0) || defined(MOTEINO_MEGA)
  #define RAMSIZE 16384
#else
  #define RAMSIZE 2048
#endif

#define MAX_BUFFER_LENGTH       61 //limit parameter update requests to 20 chars. ex: Parameter:LongRequest
#define MAX_ACK_REQUEST_LENGTH  30 //60 is max for ACK (with ATC enabled), but need to allow appending :OK and :INV to confirmations from node

typedef struct req {
  uint16_t nodeId;
  char data[MAX_BUFFER_LENGTH]; //+1 for the null terminator
  struct req *next;
}REQUEST;

//dynamically allocated queue (FIFO) data structure
REQUEST* queue = NULL;
byte size_of_queue = 0;

char buff[61]; //61 max payload for radio packets
PString Pbuff(buff, sizeof(buff)); //easy string manipulator
int rssi=0; //signed!
//******************************************** END ADVANCED variables ********************************************************************************
//******************************************** BEGIN GENERAL variables ********************************************************************************
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit Windbond FlashMEM chip
//******************************************** END GENERAL variables ********************************************************************************

void setup() {
#if defined (MOTEINO_M0) && defined(SERIAL_PORT_USBVIRTUAL)
  delay(2000);
#endif

  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  LED_HIGH;

  delay(100);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
#endif

  Serial << endl << "GATEWAYSTART" << endl;
  PRINT_FREQUENCY;
  PRINT_UPTIME;

  if (flash.initialize()) {
    DEBUGln(F("DEBUG:SPI Flash Init OK!"));
    flash.sleep();
  }
  else DEBUGln(F("DEBUG:SPI Flash MEM FAIL!"));
  LED_LOW;
}

void loop() {
  handleSerialData();   //checks for any serial input from the host computer (Pi)

  //process any received radio packets
  if (radio.receiveDone())
  {
    LED_HIGH;
    rssi = radio.RSSI; //get this asap from transceiver
    if (radio.DATALEN > 0) //data packets have a payload
    {
      for (byte i=9;i<radio.DATALEN;i++) {
        if (radio.DATA[i]=='\n' || radio.DATA[i]=='\r')
          radio.DATA[i]=' '; //remove any newlines in the payload - this should only ever happen with noise data that actually made it through
      }
      Serial << F("[") << radio.SENDERID << F("] ") << (char*)radio.DATA << " SS:" << rssi << endl; //this passes data to host computer (Pi)
    }

    //check if the packet is a wireless programming request
#ifdef ENABLE_WIRELESS_PROGRAMMING
    CheckForWirelessHEX(radio, flash, false); //non verbose DEBUG
#endif

    //respond to any ACK if requested
    if (radio.ACKRequested())
    {
      REQUEST* aux=queue;
      Pbuff="";
      //walk queue and add pending commands to ACK payload (as many it can fit)
      while (aux!=NULL) {
        if (aux->nodeId==radio.SENDERID)
        {
          //check if payload has room to add this queued command
          if (Pbuff.length() + 1 + strlen(aux->data) <= MAX_ACK_REQUEST_LENGTH)
          {
            if (Pbuff.length()) Pbuff.print(' '); //prefix with a space any previous command in buffer
            Pbuff.print(aux->data); //append command
          }
        }
        aux=aux->next;
      }
      if (Pbuff.length())
        radio.sendACK(buff, Pbuff.length());
      else
        radio.sendACK();
    }
    LED_LOW;
  }
}

boolean insert(uint16_t new_id, char new_data[]) { 
  REQUEST* aux;
  REQUEST* new_node = (REQUEST*)malloc(sizeof(REQUEST));
  if (new_node == NULL) return false;
  new_node->nodeId = new_id; 
  strcpy(new_node->data, new_data);
  new_node->next = NULL;
  if (queue == NULL) queue = new_node;
  else {
      aux = queue;
      while(aux->next != NULL) aux=aux->next;
      aux->next=new_node;
  }
  return true;
}

//processCommand - parse the command and send it to target
//if target is non-responsive it(sleeppy node?) then queue command to send when target wakes and asks for an ACK
//SPECIAL COMMANDS FROM HOST:
// - RQ:123:MESSAGE - send or (upon fail) queue message
// - 123:VOID - removes all queued commands for node 123
// - 123:VOID:command - removes 'command' from queue (if found)
// - RQ - prints the queued list of nodes on serial port, to host (Pi?)
// - RQ:VOID - flush entire queue
// - FREERAM - returns # of unallocated bytes at end of heap
// - SYSFREQ - returns operating frequency in Hz
// - UPTIME - returns millis()
void processCommand(char data[], boolean allowDuplicate=false) {
  char *ptr;
  char dataPart[MAX_BUFFER_LENGTH];
  uint16_t targetId;
  byte sendLen = 0;
  byte isQueueRequest = false;
  ptr = strtok(data, ":");

  if (strcmp(data, "FREERAM")==0)
    Serial << F("FREERAM:") << freeRAM() << ':' << RAMSIZE << endl;
  if (strcmp(data, "RQ")==0)
  {
    ptr = strtok(NULL, ":");  //move to next :
    if (ptr == NULL) printQueue(queue);
    else isQueueRequest = true;
  }
  if (strcmp(data, "SYSFREQ")==0)
    PRINT_FREQUENCY;
  if (strcmp(data, "UPTIME")==0)
    PRINT_UPTIME;
  if (strcmp(data, "NETWORKID")==0)
    Serial << F("NETWORKID:") << NETWORKID << endl;
  if (strcmp(data, "ENCRYPTKEY")==0)
#ifdef ENCRYPTKEY
    Serial << F("ENCRYPTKEY:") << ENCRYPTKEY << endl;
#else
    Serial << F("ENCRYPTKEY:NONE") << endl;
#endif

  if(ptr != NULL) {                  // delimiter found, valid command
    sprintf(dataPart, "%s", ptr);

    //if "RQ:VOID" then flush entire requst queue
    if (isQueueRequest && strcmp(dataPart, "VOID")==0) {
      REQUEST* aux = queue;
      byte removed=0;
  
      while(aux != NULL) {
        if (aux == queue) {
          if (aux->next == NULL) {
            free(queue);
            queue=NULL;
            removed++;
            break;
          }
          else {
            queue = queue->next;
            free(aux);
            removed++;
            aux = queue;
            continue;
          }
        }
      }
      DEBUG("DEBUG:VOIDED_commands:");DEBUGln(removed);
      size_of_queue = size_of_queue - removed;
      return;
    }

    targetId = atoi(dataPart);       // attempt to extract nodeID part
    ptr = strtok(NULL, "");          // get command part to the end of the string
    sprintf(dataPart, "%s", ptr);

    //check for empty command
    if (strlen(dataPart) == 0) return;

    //check target nodeID is valid
    if (targetId > 0 && targetId != NODEID && targetId<=1023) {
      REQUEST* aux;
      byte removed=0;

      //check if VOID command - if YES then remove command(s) to that target nodeID
      if (strstr(dataPart, "VOID")==dataPart) //string starts with VOID
      {
        //if 'nodeId:VOID' then remove all commands to that node
        //if 'nodeId:VOID:REQUEST' then remove just 'REQUEST' (if found & identical match)
        boolean removeAll=true;
        if (dataPart[4]==':' && strlen(dataPart)>5)
          removeAll=false;

        //iterate over queue
        aux = queue;
        while(aux != NULL) {
          if (aux->nodeId==targetId)
          {
            if (removeAll || (!removeAll && strcmp(aux->data, dataPart+5)==0))
            {
              if (aux == queue)
              {
                if (aux->next == NULL)
                {
                  free(queue);
                  queue=NULL;
                  removed++;
                  break;
                }
                else
                {
                  queue = queue->next;
                  free(aux);
                  removed++;
                  aux = queue;
                  continue;
                }
              }
              else
              {
                REQUEST* prev=queue;
                while(prev->next != NULL && prev->next != aux) prev = prev->next; //find previous
                if (prev->next == NULL) break;
                prev->next=prev->next->next;
                free(aux);
                removed++;
                aux=prev->next;
              }
            }
            else aux=aux->next;
          }
          else aux=aux->next;
        }
        DEBUG("DEBUG:VOIDED_commands:");DEBUGln(removed);
        size_of_queue = size_of_queue - removed;
        return;
      }

      //try sending to node, if it fails, continue & add to pending commands queue
      LED_HIGH;
      if (radio.sendWithRetry(targetId, dataPart, strlen(dataPart)))
      {
        LED_LOW;
        return;
      }
      LED_LOW;

      if (!isQueueRequest) return; //just return at this time if not queued request

      //check for duplicate
      if (!allowDuplicate) {
        //walk queue and check for duplicates
        aux = queue;
        while(aux != NULL)
        {
          //DEBUGln("While");
          if (aux->nodeId==targetId)
          {
            if (strcmp(aux->data, dataPart)==0)
            {
              DEBUGln(F("DEBUG:processCommand_skip_duplicate"));  
              return;
            }
          }
          aux = aux->next;
        }
      }

      //all checks OK, attempt to add to queue
      if (insert(targetId, dataPart))
      {
        //DEBUG(F("-> inserted: ")); 
        //DEBUG(targetId);
        //DEBUG("_");
        //DEBUGln(dataPart);
        size_of_queue++;
      }
      else
      {
        DEBUGln(F("DEBUG:INSERT_FAIL:MEM_FULL"));
        Serial << F("[") << targetId << F("] ") << dataPart << F(":MEMFULL") << endl;
      }
    }
    else { 
      //DEBUG(F("DEBUG:INSERT_FAIL - INVALID nodeId:")); DEBUGln(targetId);
      Serial<< '[' << targetId <<"] " << dataPart << F(":INV:ID-OUT-OF-RANGE") << endl;
    }
  }
}

void printQueue(REQUEST* p) {
  if (!size_of_queue) {
    Serial << F("RQ:EMPTY") << endl;
    return;
  }

  REQUEST* aux=p;
  while (aux!=NULL) {
    Serial << F("RQ:") << aux->nodeId << ':' << aux->data << endl;
    aux=aux->next;
  }
}

// here's the processing of single char/bytes as soon as they're coming from UART
void handleSerialData() {
  static char input_line[100]; //static = these get allocated ONCE!
  static byte input_pos = 0;
  if(Serial.available() > 0)
  {
    char inByte = Serial.read();
    switch (inByte)
    {
      case '\r':   //ignore carriage return
        break;

      case '\n':
        if (input_pos==0) break;       // ignore empty lines
        input_line[input_pos] = 0;     // null terminate the string
        DEBUG("DEBUG:handleSerialData:");
        DEBUGln(input_line);
        processCommand(input_line);        // fill up queue
        input_pos = 0; // reset buffer for next time
        break;

      default:
        // keep adding if not full ... allow for terminating byte
        if (input_pos < MAX_BUFFER_LENGTH-1) {
          input_line[input_pos] = inByte;
          input_pos++;
        } else {
          // if theres no EOL coming before MAX_BUFF_CHARS is exceeded we'll just terminate and send it, last char is then lost
          input_line[input_pos] = 0;    // null terminate the string
          DEBUG("DEBUG:MAX_BUFF_CHARS is exceeded - attempting to add (default): ");
          DEBUGln(input_line);
          processCommand(input_line);  //add to queue
          input_pos = 0; //reset buffer for next line
        }
        break;
    }
  }
}

//returns # of unfragmented free RAM bytes (free end of heap)
extern "C" char *sbrk(int i);
int freeRAM() {
#ifdef __arm__
  char top=0;
  return &top - reinterpret_cast<char*>(sbrk(0));
#else
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
#endif
}

//returns total # of free RAM bytes (all free heap, including fragmented memory)
int allFreeRAM() 
{
  int size = 1024;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}