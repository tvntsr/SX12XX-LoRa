
/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 20/09/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


#define NSS 10                                  //select on LoRa device
#define NRESET 9                                //reset on LoRa device
#define DIO0 3                                  //DIO0 on LoRa device, used for RX and TX done 
#define LED1 8                                  //On board LED, high for on

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using

//*******  Setup LoRa modem parameters here ! ***************
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
const int8_t  TXpower = 10;                     //LoRa TX power

const uint32_t TXtimeoutmS = 5000;              //mS to wait for TX to complete 
const uint32_t RXtimeoutmS = 60000;             //mS to wait for receiving a packet
const uint32_t ACKdelaymS = 0;                  //ms delay after packet actioned and ack sent 
const uint32_t ACKtimeoutDTmS = 2000;           //mS to wait for receiving an ACK and re-trying TX
const uint32_t packetdelaymS = 1000;            //mS delay between transmitted packets

const uint8_t DTSegmentSize = 128;              //number of bytes in each segment or payload
const uint16_t NetworkID = 0x3210;              //a unique identifier to go out with packet
