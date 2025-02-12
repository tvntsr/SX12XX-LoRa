/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 01/01/22

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Hardware ppin definitions ***************

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define DIO0 3                                  //DIO0 pin on LoRa device, used for sensing RX and TX done 
#define LED1 8                                  //LED used to indicate transmission
#define SDCS 30

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using

#define DISPCS 23                               //CS for ILI9341 
#define DISPDC 24                               //DC for ILI9341 
#define DISPRESET 25                            //RESET for ILI9341   
#define TOUCHCS 29                              //ILI9341 may have touch ICs, so we need to disable it, set to -1 if not fitted   

//#define ENABLESOFTWARESERIAL                  //enable this define to use a software serial port for debug output
#define RXpin 2                                 //pin number for Software serial monitor RX input into Arduino
#define TXpin 3                                 //pin number for Software serial monitor TX pitput from Arduino

#define YModemnSerial Serial                    //define Serial port to use for Ymodem transfer here
#define MonitorPort Serial1                     //define Hardware Serial monitor port to use for monitoring


//*******  Setup LoRa modem parameters here ! ***************
const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint32_t TXtimeoutmS = 5000;              //mS to wait for TX to complete
const uint32_t RXtimeoutmS = 5500;              //mS to wait for receiving a packet
const uint32_t ACKdelaymS = 0;                  //ms delay after packet actioned and ack sent
const uint32_t ACKsegtimeoutmS = 75;            //mS to wait for receiving an ACK before re-trying transmit segment
const uint32_t ACKopentimeoutmS = 250;          //mS to wait for receiving an ACK before re-trying transmit file open
const uint32_t ACKclosetimeoutmS = 250;         //mS to wait for receiving an ACK before re-trying transmit file close
const uint32_t DuplicatedelaymS = 10;           //ms delay if there has been an duplicate segment or command receipt
const uint32_t NoAckCountLimit = 250;           //if no NoAckCount exceeds this value - restart transfer
const uint32_t packetdelaymS = 0;               //mS delay between transmitted packets
const uint32_t ReceiveTimeoutmS = 2000;         //mS without receiving a packet for check if PC transfer to be done    
const uint8_t HeaderSizeMax = 12;               //max size of header in bytes, minimum size is 7 bytes
const uint8_t DataSizeMax = 245;                //max size of data array in bytes
const uint8_t DTfilenamesize = 32;              //size of DTfilename buffer
const uint16_t NetworkID = 0x3210;              //a unique identifier to go out with packet


//*******  ILI9341 Display settings here ***************

const uint8_t textscale = 3;
const byte rotation = 1;
