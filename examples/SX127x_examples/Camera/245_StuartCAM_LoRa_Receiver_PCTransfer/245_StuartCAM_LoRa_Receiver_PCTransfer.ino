/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 01/01/22

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a demonstration program that uses LoRa to receive an image taken with a
  remote OV2640 camera or ESP32CAM. The LoRa transfer is carried out using the data transfer functions
  of the SX12XX-LoRa library.

  The Arducam software (for the OV2640) on the transmitter uses program 238_StuartCAM_LoRa_Remote_Camera
  to take an image and then transfers it as a file across to this receiver, when the transfer is complete
  the file, with the original name, is saved as a file on a SD card. Then that file on the SD card is
  transferred using the YModem protocol over a serial port to a PC. The serial transfer port is assumed
  to be Serial, the same as the program upload port. The transfer can be monitored on the MonitorSerial
  port which can be a hardware serial port such as Serial1, for Arduinos that have additional hardware
  serial ports or a Software serial port.

  Program uses an ILI931 TFT display to show progress of the transfer. Arduino DUE was used in this
  example.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

//#define SDLIB                            //define SDLIB for SD.h or SDFATLIB for SDfat.h
#define SDFATLIB

#ifdef SDFATLIB
#include <SdFat.h>
SdFat SD;
File dataFile;                             //name the file instance needed for SD library routines
#endif

#ifdef SDLIB
#include <SD.h>
File dataFile;                             //name the file instance needed for SD library routines
#endif

#include <SX127XLT.h>                      //SX12XX-LoRa library
#include <ProgramLT_Definitions.h>         //part of SX12XX-LoRa library
#include "Settings.h"                      //LoRa settings etc.

#ifdef ENABLESOFTWARESERIAL
#include <SoftwareSerial.h>
SoftwareSerial MonitorSerial(RXpin, TXpin);
#endif

#ifndef ENABLESOFTWARESERIAL
#define MonitorSerial MonitorPort          //define hardware monitor port to use for monitoring
#endif

#include "YModem.h"                        //YModem used SD dataFile
#include <arrayRW.h>                       //part of SX12XX-LoRa library    

SX127XLT LoRa;                             //create an SX127XLT library instance called LoRa

#define PRINTSEGMENTNUM                    //enable this define to print segment numbers during data transfer
//#define DEBUG                            //enable this define to show data transfer debug info
#define ENABLEFILECRC                      //enable this define to use and show file CRCs
//#define DISABLEPAYLOADCRC                //enable this define if you want to disable payload CRC checking
#define ENABLEPCTRANSFER                   //enable this define for YModem transfer to PC 

#include "Adafruit_GFX.h"                  //get library here > https://github.com/adafruit/Adafruit-GFX-Library  
#include "Adafruit_ILI9341.h"              //get library here > https://github.com/adafruit/Adafruit_ILI9341

Adafruit_ILI9341 disp = Adafruit_ILI9341(DISPCS, DISPDC, DISPRESET);  //for dispaly defaults, textsze and rotation, see Settings.h

uint8_t RXPacketL;                         //length of received packet
uint8_t RXPacketType;                      //type of received packet, segment write, ACK, NACK etc
uint16_t RXErrors;                         //count of packets received with error
uint8_t RXFlags;                           //DTflags byte in header, could be used to control actions in TX and RX
uint8_t RXHeaderL;                         //length of header
uint8_t RXDataarrayL;                      //length of data array\segment
int16_t PacketRSSI;                        //stores RSSI of received packet
int8_t  PacketSNR;                         //stores signal to noise ratio of received packet
uint16_t DTDestinationFileCRC;             //CRC of complete file received
uint16_t DTSourceFileCRC;                  //CRC returned of the remote saved file
uint32_t DTDestinationFileLength;          //length of file written on the destination\receiver
uint32_t DTSourceFileLength;               //length of file at source\transmitter
uint32_t DTStartmS;                        //used for timeing transfers
bool DTFileOpened;                         //bool to flag when file has been opened
bool DTFileSaved = false;                  //bool to flag when file has been saved to SD
uint16_t DTSegment = 0;                    //current segment number
uint16_t DTSegmentNext;                    //next segment expected
uint16_t DTReceivedSegments;               //count of segments received
uint16_t DTSegmentLast;                    //last segment processed
char DTfilenamebuff[DTfilenamesize];

int DTLED = -1;                            //pin number for indicator LED, if -1 then not used

uint8_t DTheader[16];                      //header array
uint8_t DTdata[245];                       //data/segment array


void loop()
{
  uint32_t startmS = millis();

  while (((uint32_t) (millis() - startmS) < ReceiveTimeoutmS ))
  {
    receiveaPacketDT();
  }

#ifdef ENABLEPCTRANSFER
  //its been ReceiveTimeoutmS milliseconds since a packet has arrived, is there a filetransfer to do ?
  if (DTFileSaved)
  {
    DTFileSaved = false;
    MonitorSerial.println(F("File was saved - start YModem transfer to PC"));
    setCursor(0, 4);
    disp.print(F("YModemTX"));
    digitalWrite(LED1, HIGH);
    yModemSend(DTfilenamebuff, 1, 1);
    setCursor(0, 4);
    disp.print(F("        "));
    digitalWrite(LED1, LOW);
  }
#endif
}


void printheader(uint8_t *hdr, uint8_t hdrsize)
{
  MonitorSerial.print(F("HeaderBytes,"));
  MonitorSerial.print(hdrsize);
  MonitorSerial.print(F(" "));
  printArrayHEX(hdr, hdrsize);
}


void printArrayHEX(uint8_t *buff, uint32_t len)
{
  uint8_t index, buffdata;

  for (index = 0; index < len; index++)
  {
    buffdata = buff[index];
    if (buffdata < 16)
    {
      MonitorSerial.print(F("0"));
    }
    MonitorSerial.print(buffdata, HEX);
    MonitorSerial.print(F(" "));
  }

}


void readHeaderDT()
{
  // The first 6 bytes of the header contain the important stuff, so load it up
  // so we can decide what to do next.
  beginarrayRW(DTheader, 0);                      //start buffer read at location 0
  RXPacketType = arrayReadUint8();                //load the packet type
  RXFlags = arrayReadUint8();                     //initial DTflags byte, not used here
  RXHeaderL = arrayReadUint8();                   //load the header length
  RXDataarrayL = arrayReadUint8();                //load the datalength
  DTSegment = arrayReadUint16();                  //load the segment number
}


void printSourceFileDetails()
{
  MonitorSerial.print(DTfilenamebuff);
  MonitorSerial.print(F(" Source file length is "));
  MonitorSerial.print(DTSourceFileLength);
  MonitorSerial.println(F(" bytes"));
#ifdef ENABLEFILECRC
  MonitorSerial.print(F(" Source file CRC is 0x"));
  MonitorSerial.println(DTSourceFileCRC, HEX);
#endif
}


void printDestinationFileDetails()
{
  MonitorSerial.print(F("Destination file length "));
  MonitorSerial.print(DTDestinationFileLength);
  MonitorSerial.println(F(" bytes"));
  if (DTDestinationFileLength != DTSourceFileLength)
  {
    MonitorSerial.println(F("ERROR - file lengths do not match"));
  }
  else
  {
    MonitorSerial.println(F("File lengths match"));
  }

#ifdef ENABLEFILECRC
  MonitorSerial.print(F("Destination file CRC is 0x"));
  MonitorSerial.println(DTDestinationFileCRC, HEX);
  if (DTDestinationFileCRC != DTSourceFileCRC)
  {
    MonitorSerial.println(F("ERROR - file CRCs do not match"));
  }
  else
  {
    MonitorSerial.println(F("File CRCs match"));
  }
#endif
}


bool processFileClose()
{
  // Code for closing local SD file
  uint32_t transferms;

  MonitorSerial.print((char*) DTfilenamebuff);
  MonitorSerial.println(F(" File close request"));

  if (DTFileOpened)                                     //check if file has been opened, close it if it is
  {
    if (SD.exists(DTfilenamebuff))                      //check if file exists
    {
      DTSD_closeFile();
      transferms = millis() - DTStartmS;
      MonitorSerial.print(F("Transfer time "));
      MonitorSerial.print(transferms);
      MonitorSerial.print(F("mS"));
      MonitorSerial.println();
      MonitorSerial.println(F("File closed"));

      setCursor(0, 0);
      disp.print(F("Transfer finished"));
      setCursor(0, 3);
      disp.print(F("          "));
      setCursor(0, 3);
      disp.print(transferms);
      disp.print(F(" mS"));
      DTFileSaved = true;
      DTFileOpened = false;
      DTDestinationFileLength = DTSD_openFileRead(DTfilenamebuff);
#ifdef ENABLEFILECRC
      DTDestinationFileCRC = DTSD_fileCRCCCITT(DTDestinationFileLength);
#endif
      beginarrayRW(DTheader, 4);                       //start writing to array at location 12
      arrayWriteUint32(DTDestinationFileLength);       //write file length of file just written just written to ACK header
      arrayWriteUint16(DTDestinationFileCRC);          //write CRC of file just written to ACK header

      printDestinationFileDetails();
    }
  }
  else
  {
    MonitorSerial.println(F("File already closed"));
    delay(DuplicatedelaymS);
  }

  delay(ACKdelaymS);
#ifdef DEBUG
  MonitorSerial.println(F("Sending ACK"));
#endif
  DTheader[0] = DTFileCloseACK;

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }
  LoRa.sendACKDT(DTheader, DTFileCloseHeaderL, TXpower);
  if (DTLED >= 0)
  {
    digitalWrite(DTLED, LOW);
  }



  MonitorSerial.println();
#ifdef DEBUG
  DTSD_printDirectory();
  MonitorSerial.println();
  MonitorSerial.println();
#endif
  return true;
}


bool processFileOpen(uint8_t *buff, uint8_t filenamesize)
{
  // Code for opening local SD file
  DTFileSaved = false;
  beginarrayRW(DTheader, 4);                      //start buffer read at location 4
  DTSourceFileLength = arrayReadUint32();         //load the file length of the file being sent
  DTSourceFileCRC = arrayReadUint16();            //load the CRC of the file being sent
  memset(DTfilenamebuff, 0, DTfilenamesize);      //clear DTfilenamebuff to all 0s
  memcpy(DTfilenamebuff, buff, filenamesize);     //copy received DTdata into DTfilenamebuff
  MonitorSerial.print((char*) DTfilenamebuff);
  MonitorSerial.print(F(" SD File Open request"));
  MonitorSerial.println();
  printSourceFileDetails();

  if (DTSD_openNewFileWrite(DTfilenamebuff))      //open file for write at beginning, delete if it exists
  {
    MonitorSerial.print((char*) DTfilenamebuff);
    MonitorSerial.println(F(" DT File Opened OK"));
    MonitorSerial.println(F("Waiting transfer"));
    DTSegmentNext = 0;                            //since file is opened the next sequence should be the first
    DTFileOpened = true;
    DTStartmS = millis();
  }
  else
  {
    MonitorSerial.print((char*) DTfilenamebuff);
    MonitorSerial.println(F(" File Open fail"));
    DTFileOpened = false;
    return false;
  }

  setCursor(0, 0);
  disp.print(F("Transfer started "));
  setCursor(0, 1);
  disp.print(F("             "));                    //clear previous filename
  setCursor(0, 1);
  disp.print(DTfilenamebuff);
  setCursor(0, 2);
  disp.print(F("            "));                     //clear previous file size
  setCursor(0, 2);
  disp.print(DTSourceFileLength);
  disp.print(F(" bytes"));
  setCursor(0, 3);
  disp.print(F("          "));                       //clear transfer time
  setCursor(0, 5);
  disp.print(F("    "));

  DTStartmS = millis();
  delay(ACKdelaymS);
#ifdef DEBUG
  MonitorSerial.println(F("Sending ACK"));
#endif
  DTheader[0] = DTFileOpenACK;                      //set the ACK packet type

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }
  LoRa.sendACKDT(DTheader, DTFileOpenHeaderL, TXpower);
  if (DTLED >= 0)
  {
    digitalWrite(DTLED, LOW);
  }
  DTSegmentNext = 0;                                //after file open, segment 0 is next

  return true;
}


bool processSegmentWrite()
{
  // Code for dealing with segment writes
  // checks that the sequence of segment writes is correct

  if (!DTFileOpened)
  {
    //something is wrong, have received a request to write a segment but there is no file opened
    //need to reject the segment write with a restart NACK
    MonitorSerial.println();
    MonitorSerial.println(F("***************************************************"));
    MonitorSerial.println(F("Error - Segment write with no file open - send NACK"));
    MonitorSerial.println(F("***************************************************"));
    MonitorSerial.println();
    DTheader[0] = DTStartNACK;
    delay(ACKdelaymS);
    delay(DuplicatedelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTStartHeaderL, TXpower);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }

  if (DTSegment == DTSegmentNext)
  {
    DTSD_writeSegmentFile(DTdata, RXDataarrayL);

#ifdef PRINTSEGMENTNUM
    MonitorSerial.println(DTSegment);
    //only enable Segment number prints if using a fast Arduino, if using a slow Arduino such
    //as a 8Mhz Pro Mini, the prints to display will cause missed segments
    //setCursor(0, 7);
    //disp.print(F("    "));
    //setCursor(0, 7);
    //disp.print(DTSegment);
#endif

#ifdef DEBUG
    MonitorSerial.print(F("  Bytes,"));
    MonitorSerial.print(RXDataarrayL);
    printPacketRSSI();
    MonitorSerial.println(F(" SendACK"));
#endif

    DTheader[0] = DTSegmentWriteACK;
    delay(ACKdelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    DTReceivedSegments++;
    DTSegmentLast = DTSegment;                  //so we can tell if sequece has been received twice
    DTSegmentNext = DTSegment + 1;
    return true;
  }

  if (DTSegment == DTSegmentLast)
  {
    MonitorSerial.print(F("ERROR segment "));
    MonitorSerial.print(DTSegment);
    MonitorSerial.println(F(" already received "));
    delay(DuplicatedelaymS);
#ifdef DEBUG
    printPacketDetails();
    printPacketRSSI();
#endif
    DTheader[0] = DTSegmentWriteACK;
    delay(ACKdelaymS);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return true;
  }

  if (DTSegment != DTSegmentNext )
  {
    MonitorSerial.print(F(" ERROR Received Segment "));
    MonitorSerial.print(DTSegment);
    MonitorSerial.print(F(" expected "));
    MonitorSerial.print(DTSegmentNext);
    MonitorSerial.print(F(" "));

#ifdef DEBUG
    printPacketDetails();
    printPacketRSSI();
#endif

    DTheader[0] = DTSegmentWriteNACK;
    DTheader[4] = lowByte(DTSegmentNext);
    DTheader[5] = highByte(DTSegmentNext);
    MonitorSerial.print(F(" Send NACK for segment "));
    MonitorSerial.print(DTSegmentNext);
    delay(ACKdelaymS);
    delay(DuplicatedelaymS);                   //add an extra delay here to stop repeated segment sends
    MonitorSerial.println();
    MonitorSerial.println();
    MonitorSerial.println(F("*****************************************"));
    MonitorSerial.print(F("Transmit restart request for segment "));
    MonitorSerial.println(DTSegmentNext);
    printheader(DTheader, RXHeaderL);
    MonitorSerial.println();
    MonitorSerial.println(F("*****************************************"));
    MonitorSerial.println();
    MonitorSerial.flush();
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, HIGH);
    }
    LoRa.sendACKDT(DTheader, DTSegmentWriteHeaderL, TXpower);
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }
  return true;
}


void printPacketDetails()
{
  PacketRSSI = LoRa.readPacketRSSI();
  PacketSNR = LoRa.readPacketSNR();
  MonitorSerial.print(F(" RSSI,"));
  MonitorSerial.print(PacketRSSI);
  MonitorSerial.print(F("dBm"));

#ifdef DEBUG
  MonitorSerial.print(F(",SNR,"));
  MonitorSerial.print(PacketSNR);
  MonitorSerial.print(F("dBm,RXOKCount,"));
  MonitorSerial.print(DTReceivedSegments);
  MonitorSerial.print(F(",RXErrs,"));
  MonitorSerial.print(RXErrors);
  MonitorSerial.print(F(" RX"));
  printheader(DTheader, RXHeaderL);
#endif
}


void printPacketRSSI()
{
  PacketRSSI = LoRa.readPacketRSSI();
  MonitorSerial.print(F(" RSSI,"));
  MonitorSerial.print(PacketRSSI);
  MonitorSerial.print(F("dBm"));
}


void printSeconds()
{
  float secs;
  secs = ( (float) millis() / 1000);
  MonitorSerial.print(secs, 2);
  MonitorSerial.print(F(" "));
}


bool processPacket(uint8_t packettype)
{
  // Decide what to do with an incoming packet

  if (packettype == DTSegmentWrite)
  {
    processSegmentWrite();
    return true;
  }

  if (packettype == DTFileOpen)
  {
    processFileOpen(DTdata, RXDataarrayL);
    return true;
  }

  if (packettype == DTFileClose)
  {
    processFileClose();
    return true;
  }
  return true;
}


bool receiveaPacketDT()
{
  // Receive Data transfer packets

  RXPacketType = 0;
  RXPacketL = LoRa.receiveDT(DTheader, HeaderSizeMax, (uint8_t *) DTdata, DataSizeMax, NetworkID, RXtimeoutmS, WAIT_RX);

  if (DTLED >= 0)
  {
    digitalWrite(DTLED, HIGH);
  }

#ifdef DEBUG
  printSeconds();
#endif

  if (RXPacketL > 0)
  {
    //if the LT.receiveDT() returns a value > 0 for RXPacketL then packet was received OK
    //then only action payload if destinationNode = thisNode
    readHeaderDT();                      //get the basic header details into global variables RXPacketType etc
    processPacket(RXPacketType);         //process and act on the packet
    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return true;
  }
  else
  {
    //if the LoRa.receiveDT() function detects an error RXOK is 0

    RXErrors++;
#ifdef DEBUG
    MonitorSerial.print(F("PacketError"));
    printPacketDetails();
    LoRa.printReliableStatus();
    LoRa.printIrqStatus();
    MonitorSerial.println();
#endif

    if (DTLED >= 0)
    {
      digitalWrite(DTLED, LOW);
    }
    return false;
  }
}


void setDTLED(int8_t pinnumber)
{
  if (pinnumber >= 0)
  {
    DTLED = pinnumber;
    pinMode(pinnumber, OUTPUT);
  }
}


bool DTSD_initSD(uint8_t CSpin)
{
  if (SD.begin(CSpin))
  {
    return true;
  }
  else
  {
    return false;
  }
}


#ifdef SDFATLIB
void DTSD_printDirectory()
{
  dataFile = SD.open("/");
  MonitorSerial.println(F("Card directory"));
  SD.ls("/", LS_R);
}
#endif


#ifdef SDLIB
void DTSD_printDirectory()
{
  dataFile = SD.open("/");

  MonitorSerial.println(F("Card directory"));

  while (true)
  {
    File entry =  dataFile.openNextFile();
    if (! entry)
    {
      //no more files
      break;
    }
    MonitorSerial.print(entry.name());
    if (entry.isDirectory())
    {
      MonitorSerial.println(F("/"));
      DTSD_printDirectory();
    }
    else
    {
      //files have sizes, directories do not
      MonitorSerial.print(F("\t\t"));
      MonitorSerial.println(entry.size(), DEC);
    }
    entry.close();
  }
  MonitorSerial.println();
}
#endif


uint32_t DTSD_openFileRead(char *buff)
{
  uint32_t filesize;

  dataFile = SD.open(buff);
  filesize = dataFile.size();
  dataFile.seek(0);
  return filesize;
}


bool DTSD_openNewFileWrite(char *buff)
{
  if (SD.exists(buff))
  {
    MonitorSerial.print(buff);
    MonitorSerial.println(F(" File exists - deleting"));
    SD.remove(buff);
  }

  if (dataFile = SD.open(buff, FILE_WRITE))
  {
    MonitorSerial.print(buff);
    MonitorSerial.println(F(" SD File opened"));
    return true;
  }
  else
  {
    MonitorSerial.print(buff);
    MonitorSerial.println(F(" ERROR opening file"));
    return false;
  }
}


uint8_t DTSD_writeSegmentFile(uint8_t *buff, uint8_t segmentsize)
{
  uint8_t index, byteswritten = 0;

  for (index = 0; index < segmentsize; index++)
  {
    dataFile.write(buff[index]);
    byteswritten++;
  }
  return byteswritten;
}


void DTSD_seekFileLocation(uint32_t position)
{
  dataFile.seek(position);                             //seek to position in file
  return;
}


void DTSD_closeFile()
{
  dataFile.close();                                   //close local file
}


void setCursor(uint8_t lcol, uint8_t lrow)
{
  disp.setCursor((lcol * 6 * textscale), (lrow * 9 * textscale));
}


uint16_t DTSD_fileCRCCCITT(uint32_t fsize)
{
  uint32_t index;
  uint16_t CRCcalc;
  uint8_t j, filedata;

  CRCcalc = 0xFFFF;                                  //start value for CRC16

  for (index = 0; index < fsize; index++)
  {
    filedata = dataFile.read();
    CRCcalc ^= (((uint16_t) filedata ) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRCcalc & 0x8000)
        CRCcalc = (CRCcalc << 1) ^ 0x1021;
      else
        CRCcalc <<= 1;
    }
  }
  return CRCcalc;
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);                       //setup pin as output for indicator LED
  led_Flash(2, 125);                           //two quick LED flashes to indicate program start
  setDTLED(LED1);                              //setup LED pin for data transfer indicator

  digitalWrite(DISPCS, HIGH);
  pinMode(DISPCS, OUTPUT);                     //disable ILI9341 for now
  digitalWrite(NSS, HIGH);
  pinMode(NSS, OUTPUT);                        //disable LoRa device for now

  if (TOUCHCS >= 0)
  {
    digitalWrite(TOUCHCS, HIGH);               //disable touch IC on ILI9341
    pinMode(TOUCHCS, OUTPUT);
  }

  YModemnSerial.begin(115200);
  MonitorSerial.begin(9600);
  MonitorSerial.println();
  MonitorSerial.println(__FILE__);
  MonitorSerial.println();

  SPI.begin();

  if (LoRa.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    MonitorSerial.println(F("LoRa device error"));
    while (1)
    {
      led_Flash(50, 50);                          //long fast speed flash indicates device error
    }
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  MonitorSerial.print(F("Initializing SD card..."));

  if (DTSD_initSD(SDCS))
  {
    MonitorSerial.println(F("SD Card initialized."));
  }
  else
  {
    MonitorSerial.println(F("SD Card failed, or not present."));
    while (1) led_Flash(100, 50);
  }

  MonitorSerial.println();

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    MonitorSerial.println(F("Payload CRC disabled"));
  }
  else
  {
    MonitorSerial.println(F("Payload CRC enabled"));
  }

  disp.begin();
  disp.fillScreen(ILI9341_BLACK);
  disp.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  disp.setRotation(rotation);
  disp.setTextSize(textscale);
  setCursor(0, 0);
  disp.print(F("Waiting Transfer"));

  DTSegmentNext = 0;
  DTFileOpened = false;

  Serial.println(F("LoRa file transfer receiver ready"));
}
