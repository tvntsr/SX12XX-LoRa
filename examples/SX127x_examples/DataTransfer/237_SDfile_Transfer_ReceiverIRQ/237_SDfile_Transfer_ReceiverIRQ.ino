/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 09/11/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a test program for the use of a data transfer (DT) packet to send a file
  from the SD card on one Arduino to the SD card on another Arduino, Arduino DUEs were used for the test.

  DT packets can be used for transfering large amounts of data in a sequence of packets or segments,
  in a reliable and resiliant way. The remote file open request, the segements sent and the remote file close
  will be transmitted until a valid acknowledge comes from the receiver. Use with the matching transmitter
  program, 233_LoRa_SDfile_Transfer_Transmitter.ino.

  Each DT packet contains a variable length header array and a variable length data array as the payload.
  On transmission the NetworkID and CRC of the payload are appended to the end of the packet by the library
  routines. The use of a NetworkID and CRC ensures that the receiver can validate the packet to a high degree
  of certainty. The receiver will not accept packets that dont have the appropriate NetworkID or payload CRC
  at the end of the packet.

  The transmitter sends a sequence of segments in order and the receiver keeps track of the sequence. If
  the sequence fails for some reason, the receiver will return a NACK packet to the transmitter requesting
  the segment sequence it was expecting.

  Details of the packet identifiers, header and data lengths and formats used are in the file
  Data_transfer_packet_definitions.md in the \SX127X_examples\DataTransfer\ folder.

  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#include <SPI.h>

#include <SX127XLT.h>
#include <ProgramLT_Definitions.h>
#include "DTSettings.h"                     //LoRa settings etc.
#include <arrayRW.h>

SX127XLT LoRa;                              //create an SX127XLT library instance called LoRa

//#define SDLIB                             //define SDLIB for SD.h or SDFATLIB for SDfat.h
#define SDFATLIB

#define PRINTSEGMENTNUM
//#define DEBUG
//#define DEBUGSD
//#define ENABLEFILECRC                      //enable this define to uses and show file CRCs
//#define DISABLEPAYLOADCRC                  //enable this define if you want to disable payload CRC checking

#include "DTSDlibrary.h"
#include "DTLibraryIRQ.h"


void loop()
{
  receiveaPacketDT();
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

  Serial.begin(115200);
  Serial.println();
  Serial.println(F(__FILE__));

  SPI.begin();

  if (LoRa.begin(NSS, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("LoRa device error"));
    while (1)
    {
      led_Flash(50, 50);                          //long fast speed flash indicates device error
    }
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  LoRa.printOperatingSettings();
  Serial.println();
  LoRa.printModemSettings();
  Serial.println();

  Serial.print(F("Initializing SD card..."));

  if (DTSD_initSD(SDCS))
  {
    Serial.println(F("SD Card initialized."));
  }
  else
  {
    Serial.println(F("SD Card failed, or not present."));
    while (1) led_Flash(100, 50);
  }

  Serial.println();
  DTSD_printDirectory();
  Serial.println();

#ifdef DISABLEPAYLOADCRC
  LoRa.setReliableConfig(NoReliableCRC);
#endif

  if (LoRa.getReliableConfig(NoReliableCRC))
  {
    Serial.println(F("Payload CRC disabled"));
  }
  else
  {
    Serial.println(F("Payload CRC enabled"));
  }

  DTSegmentNext = 0;
  DTFileOpened = false;

  Serial.println(F("SDfile transfer receiver ready"));
  Serial.println();


}
