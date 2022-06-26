// AP_Lab6.c
// Runs on either MSP432 or TM4C123
// see GPIO.c file for hardware connections

// Daniel Valvano and Jonathan Valvano
// November 20, 2016
// CC2650 booster or CC2650 LaunchPad, CC2650 needs to be running SimpleNP 2.2 (POWERSAVE)

#include <stdint.h>
#include <string.h>
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/AP.h"
#include "AP_Lab6.h"

//**debug macros**APDEBUG defined in AP.h********
#ifdef APDEBUG
#define OutString(STRING) UART0_OutString(STRING)
#define OutUHex(NUM) UART0_OutUHex(NUM)
#define OutUHex2(NUM) UART0_OutUHex2(NUM)
#define OutChar(N) UART0_OutChar(N)
#else
#define OutString(STRING)
#define OutUHex(NUM)
#define OutUHex2(NUM)
#define OutChar(N)
#endif

//****links into AP.c**************
extern const uint32_t RECVSIZE;
extern uint8_t RecvBuf[];

typedef struct characteristics
{
  uint16_t theHandle;          // each object has an ID
  uint16_t size;               // number of bytes in user data (1,2,4,8)
  uint8_t *pt;                 // pointer to user data, stored little endian
  void (*callBackRead)(void);  // action if SNP Characteristic Read Indication
  void (*callBackWrite)(void); // action if SNP Characteristic Write Indication
} characteristic_t;

extern const uint32_t MAXCHARACTERISTICS;
extern uint32_t CharacteristicCount;
extern characteristic_t CharacteristicList[];

typedef struct NotifyCharacteristics
{
  uint16_t uuid;              // user defined
  uint16_t theHandle;         // each object has an ID (used to notify)
  uint16_t CCCDhandle;        // generated/assigned by SNP
  uint16_t CCCDvalue;         // sent by phone to this object
  uint16_t size;              // number of bytes in user data (1,2,4,8)
  uint8_t *pt;                // pointer to user data array, stored little endian
  void (*callBackCCCD)(void); // action if SNP CCCD Updated Indication
} NotifyCharacteristic_t;

extern const uint32_t NOTIFYMAXCHARACTERISTICS;
extern uint32_t NotifyCharacteristicCount;
extern NotifyCharacteristic_t NotifyCharacteristicList[];
//**************Lab 6 routines*******************
// **********SetFCS**************
// helper function, add check byte to message
// assumes every byte in the message has been set except the FCS
// used the length field, assumes less than 256 bytes
// FCS = 8-bit EOR(all bytes except SOF and the FCS itself)
// Inputs: pointer to message
//         stores the FCS into message itself
// Outputs: none
void SetFCS(uint8_t *msg)
{
  // read the length from the length field, the 2nd bytes
  const uint32_t length = AP_GetSize(msg);

  uint8_t fcs = 0;

  // fcs starts from the 2-bytes length field, 2-bytes command field
  // the N-bytes data field
  uint8_t idx = 0;
  for ( idx = 1; idx <= (4 + length); idx++)
  {
    // fcs is XOR
    fcs = fcs ^ msg[idx];
  }

  // set the fcs
  msg[idx] = fcs;
}
//*************BuildGetStatusMsg**************
// Create a Get Status message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void BuildGetStatusMsg(uint8_t *msg)
{
  msg[0] = SOF;
  msg[1] = 0x00;
  msg[2] = 0x00;
  msg[3] = 0x55;
  msg[4] = 0x06;
  SetFCS(msg);
}
//*************Lab6_GetStatus**************
// Get status of connection, used in Lab 6
// Input:  none
// Output: status 0xAABBCCDD
// AA is GAPRole Status
// BB is Advertising Status
// CC is ATT Status
// DD is ATT method in progress
uint32_t Lab6_GetStatus(void)
{
  volatile int r;
  uint8_t sendMsg[8];
  OutString("\n\rGet Status");
  BuildGetStatusMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  return (RecvBuf[4] << 24) + (RecvBuf[5] << 16) + (RecvBuf[6] << 8) + (RecvBuf[7]);
}

//*************BuildGetVersionMsg**************
// Create a Get Version message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will Get Status
void BuildGetVersionMsg(uint8_t *msg)
{
  // hint: see NPI_GetVersion in AP.c
  msg[0] = SOF;
  msg[1] = 0x00;
  msg[2] = 0x00;
  msg[3] = 0x35;
  msg[4] = 0x03;
  SetFCS(msg);
}
//*************Lab6_GetVersion**************
// Get version of the SNP application running on the CC2650, used in Lab 6
// Input:  none
// Output: version
uint32_t Lab6_GetVersion(void)
{
  volatile int r;
  uint8_t sendMsg[8];
  OutString("\n\rGet Version");
  BuildGetVersionMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  return (RecvBuf[5] << 8) + (RecvBuf[6]);
}

//*************BuildAddServiceMsg**************
// Create an Add service message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        pointer to empty buffer of at least 9 bytes
// Output none
// build the necessary NPI message that will add a service
void BuildAddServiceMsg(uint16_t uuid, uint8_t *msg)
{
  msg[0] = SOF;

  // length = 3 bytes
  msg[1] = 0x03; 
  msg[2] = 0x00;

  // command 
  msg[3] = 0x35;
  msg[4] = 0x81;

  // message - 3 bytes
  msg[5] = 0x01; //primary service
  msg[6] = (uint8_t)uuid;
  msg[7] = (uint8_t)(uuid >> 8);

  // set the FCS
  SetFCS(msg);
}

//*************Lab6_AddService**************
// Add a service, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
// Output APOK if successful,
//        APFAIL if SNP failure
int Lab6_AddService(uint16_t uuid)
{
  int r;
  uint8_t sendMsg[12];
  OutString("\n\rAdd service");
  BuildAddServiceMsg(uuid, sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  return r;
}
//*************AP_BuildRegisterServiceMsg**************
// Create a register service message, used in Lab 6
// Inputs pointer to empty buffer of at least 6 bytes
// Output none
// build the necessary NPI message that will register a service
void BuildRegisterServiceMsg(uint8_t *msg)
{
  msg[0] = SOF;

  // length = 0 bytes
  msg[1] = 0x00; 
  msg[2] = 0x00;

  // command  - SNP register service 0x35, 0x84
  msg[3] = 0x35;
  msg[4] = 0x84;
  
  SetFCS(msg);
}
//*************Lab6_RegisterService**************
// Register a service, used in Lab 6
// Inputs none
// Output APOK if successful,
//        APFAIL if SNP failure
int Lab6_RegisterService(void)
{
  int r;
  uint8_t sendMsg[8];
  OutString("\n\rRegister service");
  BuildRegisterServiceMsg(sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  return r;
}

//*************BuildAddCharValueMsg**************
// Create a Add Characteristic Value Declaration message, used in Lab 6
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        permission is GATT Permission, 0=none,1=read,2=write, 3=Read+write
//        properties is GATT Properties, 2=read,8=write,0x0A=read+write, 0x10=notify
//        pointer to empty buffer of at least 14 bytes
// Output none
// build the necessary NPI message that will add a characteristic value
void BuildAddCharValueMsg(uint16_t uuid,
                          uint8_t permission, 
                          uint8_t properties, 
                          uint8_t *msg)
{
  msg[0] = SOF;

  // length = 8 bytes
  msg[1] = 0x08; 
  msg[2] = 0x00;

  // command  - SNP add characteristics
  msg[3] = 0x35;
  msg[4] = 0x82;

  // Permission & properties
  msg[5] = permission;
  msg[6] = properties;
  msg[7] = 0x00;

  // set RFU to 0
  msg[8] = 0;
  
  // maximum length - 512 bytes
  msg[9] = 0;
  msg[10] = 2;

  // UUID
  msg[11] = (uint8_t)uuid;
  msg[12] = (uint8_t)(uuid >> 8);

  SetFCS(msg);

  // set RFU to 0 and
  // set the maximum length of the attribute value=512
  // for a hint see NPI_AddCharValue in AP.c
  // for a hint see first half of AP_AddCharacteristic and first half of AP_AddNotifyCharacteristic
}

//*************BuildAddCharDescriptorMsg**************
// Create a Add Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least 32 bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void BuildAddCharDescriptorMsg(char name[], uint8_t *msg)
{
  // set the string
  const uint8_t len = strlen(name) + 1;

  msg[0] = SOF;

  // set command - SNP add char descriptor
  msg[3] = 0x35;
  msg[4] = 0x83;

  // set user description string
  msg[5] = 0x80;

  // set read permission
  msg[6] = 0x01;

  // set initial length and maxlength to the string length
  msg[7]  = len;
  msg[8]  = 0x00;
  msg[9]  = len;
  msg[10] = 0x00;

  // set the string
  memcpy(&msg[11], name, len);

  // set length
  msg[1] = len + 6;
  msg[2] = 0x00;

  SetFCS(msg);

  // set the permissions on the string to read
  // for a hint see NPI_AddCharDescriptor in AP.c
  // for a hint see second half of AP_AddCharacteristic
}

//*************Lab6_AddCharacteristic**************
// Add a read, write, or read/write characteristic, used in Lab 6
//        for notify properties, call AP_AddNotifyCharacteristic
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        thesize is the number of bytes in the user data 1,2,4, or 8
//        pt is a pointer to the user data, stored little endian
//        permission is GATT Permission, 0=none,1=read,2=write, 3=Read+write
//        properties is GATT Properties, 2=read,8=write,0x0A=read+write
//        name is a null-terminated string, maximum length of name is 20 bytes
//        (*ReadFunc) called before it responses with data from internal structure
//        (*WriteFunc) called after it accepts data into internal structure
// Output APOK if successful,
//        APFAIL if name is empty, more than 8 characteristics, or if SNP failure
int Lab6_AddCharacteristic(uint16_t uuid, uint16_t thesize, void *pt, uint8_t permission,
                           uint8_t properties, char name[], void (*ReadFunc)(void), void (*WriteFunc)(void))
{
  int r;
  uint16_t handle;
  uint8_t sendMsg[32];
  if (thesize > 8)
    return APFAIL;
  if (name[0] == 0)
    return APFAIL; // empty name
  if (CharacteristicCount >= MAXCHARACTERISTICS)
    return APFAIL; // error
  BuildAddCharValueMsg(uuid, permission, properties, sendMsg);
  OutString("\n\rAdd CharValue");
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  if (r == APFAIL)
    return APFAIL;
  handle = (RecvBuf[7] << 8) + RecvBuf[6]; // handle for this characteristic
  OutString("\n\rAdd CharDescriptor");
  BuildAddCharDescriptorMsg(name, sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  if (r == APFAIL)
    return APFAIL;
  CharacteristicList[CharacteristicCount].theHandle = handle;
  CharacteristicList[CharacteristicCount].size = thesize;
  CharacteristicList[CharacteristicCount].pt = (uint8_t *)pt;
  CharacteristicList[CharacteristicCount].callBackRead = ReadFunc;
  CharacteristicList[CharacteristicCount].callBackWrite = WriteFunc;
  CharacteristicCount++;
  return APOK; // OK
}

//*************BuildAddNotifyCharDescriptorMsg**************
// Create a Add Notify Characteristic Descriptor Declaration message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 20 bytes
//        pointer to empty buffer of at least bytes
// Output none
// build the necessary NPI message that will add a Descriptor Declaration
void BuildAddNotifyCharDescriptorMsg(char name[], uint8_t *msg)
{
  // set the string
  const uint8_t len = strlen(name) + 1;

  msg[0] = SOF;

  // set command - SNP add char descriptor
  msg[3] = 0x35;
  msg[4] = 0x83;

  // set user description string + CCCD
  msg[5] = 0x84;

  // set CCCD read + write permission
  msg[6] = 0x03;

  // set GATT permission read
  msg[7] = 0x01;

  // set initial length and maxlength to the string length
  msg[8]  = len;
  msg[9]  = 0x00;
  msg[10] = len;
  msg[11] = 0x00;

  // set the string
  memcpy(&msg[12], name, len);

  // set length
  msg[1] = len + 7;
  msg[2] = 0x00;

  SetFCS(msg);
  // set length and maxlength to the string length
  // set the permissions on the string to read
  // set User Description String
  // set CCCD parameters read+write
  // for a hint see NPI_AddCharDescriptor4 in VerySimpleApplicationProcessor.c
  // for a hint see second half of AP_AddNotifyCharacteristic
}

//*************Lab6_AddNotifyCharacteristic**************
// Add a notify characteristic, used in Lab 6
//        for read, write, or read/write characteristic, call AP_AddCharacteristic
// Inputs uuid is 0xFFF0, 0xFFF1, ...
//        thesize is the number of bytes in the user data 1,2,4, or 8
//        pt is a pointer to the user data, stored little endian
//        name is a null-terminated string, maximum length of name is 20 bytes
//        (*CCCDfunc) called after it accepts , changing CCCDvalue
// Output APOK if successful,
//        APFAIL if name is empty, more than 4 notify characteristics, or if SNP failure
int Lab6_AddNotifyCharacteristic(uint16_t uuid, uint16_t thesize, void *pt,
                                 char name[], void (*CCCDfunc)(void))
{
  int r;
  uint16_t handle;
  uint8_t sendMsg[36];
  if (thesize > 8)
    return APFAIL;
  if (NotifyCharacteristicCount >= NOTIFYMAXCHARACTERISTICS)
    return APFAIL; // error
  BuildAddCharValueMsg(uuid, 0, 0x10, sendMsg);
  OutString("\n\rAdd Notify CharValue");
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  if (r == APFAIL)
    return APFAIL;
  handle = (RecvBuf[7] << 8) + RecvBuf[6]; // handle for this characteristic
  OutString("\n\rAdd CharDescriptor");
  BuildAddNotifyCharDescriptorMsg(name, sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  if (r == APFAIL)
    return APFAIL;
  NotifyCharacteristicList[NotifyCharacteristicCount].uuid = uuid;
  NotifyCharacteristicList[NotifyCharacteristicCount].theHandle = handle;
  NotifyCharacteristicList[NotifyCharacteristicCount].CCCDhandle = (RecvBuf[8] << 8) + RecvBuf[7]; // handle for this CCCD
  NotifyCharacteristicList[NotifyCharacteristicCount].CCCDvalue = 0;                               // notify initially off
  NotifyCharacteristicList[NotifyCharacteristicCount].size = thesize;
  NotifyCharacteristicList[NotifyCharacteristicCount].pt = (uint8_t *)pt;
  NotifyCharacteristicList[NotifyCharacteristicCount].callBackCCCD = CCCDfunc;
  NotifyCharacteristicCount++;
  return APOK; // OK
}

//*************BuildSetDeviceNameMsg**************
// Create a Set GATT Parameter message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message to set Device name
void BuildSetDeviceNameMsg(char name[], uint8_t *msg)
{
  // for a hint see NPI_GATTSetDeviceNameMsg in VerySimpleApplicationProcessor.c
  // for a hint see NPI_GATTSetDeviceName in AP.c
  
  // len
  const uint8_t len = strlen(name);

  // sof
  msg[0] = SOF;

  // SNP Sett GATT parameter
  msg[3] = 0x35;
  msg[4] = 0x8C;

  // Generic Access Service
  msg[5] = 0x01;

  // name
  msg[6] = 0x00;
  msg[7] = 0x00;
  memcpy(&msg[8], name, len);

  // length field
  msg[1] = len + 3;
  msg[2] = 0;

  SetFCS(msg);
}
//*************BuildSetAdvertisementData1Msg**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs pointer to empty buffer of at least 16 bytes
// Output none
// build the necessary NPI message for Non-connectable Advertisement Data
void BuildSetAdvertisementData1Msg(uint8_t *msg)
{
  // for a hint see NPI_SetAdvertisementMsg in VerySimpleApplicationProcessor.c
  // for a hint see NPI_SetAdvertisement1 in AP.c
  // Non-connectable Advertisement Data
  // GAP_ADTYPE_FLAGS,DISCOVERABLE | no BREDR
  // Texas Instruments Company ID 0x000D
  // TI_ST_DEVICE_ID = 3
  // TI_ST_KEY_DATA_ID
  // Key state=0
  
  msg[0] = SOF;

  // SNP set Advertisement data
  msg[3] = 0x55;
  msg[4] = 0x43;

  // Not connected advertisement data
  msg[5] = 0x01;

  // GAP_ADTYPE_FLAGS, DISCOVERABLE | no BREDR
  msg[6] = 0x02;
  msg[7] = 0x01;
  msg[8] = 0x06;

  // length, manu sepcific
  msg[9] = 0x06;
  msg[10] = 0xFF;

  // texas isntruements company id
  msg[11] = 0x0D;
  msg[12] = 0x00;
  
  msg[13] = 0x03,           // TI_ST_DEVICE_ID
  msg[14] = 0x00,           // TI_ST_KEY_DATA_ID
  msg[15] = 0x00,           // Key state

  msg[1] = 11;
  msg[2] = 0;

  SetFCS(msg);
}

//*************BuildSetAdvertisementDataMsg**************
// Create a Set Advertisement Data message, used in Lab 6
// Inputs name is a null-terminated string, maximum length of name is 24 bytes
//        pointer to empty buffer of at least 36 bytes
// Output none
// build the necessary NPI message for Scan Response Data
void BuildSetAdvertisementDataMsg(char name[], uint8_t *msg)
{
  // for a hint see NPI_SetAdvertisementDataMsg in VerySimpleApplicationProcessor.c
  // for a hint see NPI_SetAdvertisementData in AP.c
  // len
  const uint8_t len = strlen(name);

  msg[0] = SOF;

  // SNP set advertisement data
  msg[3] = 0x55;
  msg[4] = 0x43;

  // scan response data
  msg[5] = 0x00;

  // len, include null
  msg[6] = len + 1;

  // type = LOCAL_NAME_COMPLETE
  msg[7] = 0x09;

  // string
  memcpy(&msg[8], name, len);

  const uint8_t idx = 8 + len;
  
  // length of this data
  msg[idx] = 0x05;

  // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  msg[idx + 1] = 0x12;

  // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  msg[idx + 2] = 0x50;
  msg[idx + 3] = 0x00;

  // DEFAULT_DESIRED_MAX_CONN_INTERVAL
  msg[idx + 4] = 0x20;
  msg[idx + 5] = 0x03;

  /*** tx power level   ****/
  // length of this data
  msg[idx + 6] = 0x02;           
  
  // GAP_ADTYPE_POWER_LEVEL
  msg[idx + 7] = 0x0A;           
  
  // 0dBm
  msg[idx + 8] = 0x00;           
  
  // length field, exclude sof, 2-bytes length and 2-bytes cmd
  msg[1] = (idx + 9) - 5;
  msg[2] = 0;

  SetFCS(msg);
}
//*************BuildStartAdvertisementMsg**************
// Create a Start Advertisement Data message, used in Lab 6
// Inputs advertising interval
//        pointer to empty buffer of at least 20 bytes
// Output none
// build the necessary NPI message to start advertisement
void BuildStartAdvertisementMsg(uint16_t interval, uint8_t *msg)
{
  // for a hint see NPI_StartAdvertisementMsg in VerySimpleApplicationProcessor.c
  // for a hint see NPI_StartAdvertisement in AP.c
  
  msg[0] = SOF;

  // SNP start advertisement
  msg[3] = 0x55;
  msg[4] = 0x42;

  msg[5] = 0x00; // Connectable Undirected Advertisements
  msg[6] = 0x00;
  msg[7] = 0x00;  // Advertise infinitely.

   // Advertising Interval (100 * 0.625 ms=62.5ms)
  msg[8] = interval;
  msg[9] = (uint8_t)(interval >> 8); 

  // Filter Policy RFU
  msg[10] = 0x00; 
  msg[11] = 0x00; 
  
  // Initiator Address Type RFU
  msg[12] = 0x00;
  msg[13] = 0x01;
  msg[14] = 0x00;
  msg[15] = 0x00;
  msg[16] = 0x00;
  msg[17] = 0xC5; // RFU

  // Advertising will restart with connectable advertising when a connection is terminated
  msg[18] = 0x02; 

  msg[1] = 14;
  msg[2] = 0;
  SetFCS(msg);
}

//*************Lab6_StartAdvertisement**************
// Start advertisement, used in Lab 6
// Input:  none
// Output: APOK if successful,
//         APFAIL if notification not configured, or if SNP failure
int Lab6_StartAdvertisement(void)
{
  volatile int r;
  uint8_t sendMsg[40];
  OutString("\n\rSet Device name");
  BuildSetDeviceNameMsg("Shape the World", sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  OutString("\n\rSetAdvertisement1");
  BuildSetAdvertisementData1Msg(sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  OutString("\n\rSetAdvertisement Data");
  BuildSetAdvertisementDataMsg("Shape the World", sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  OutString("\n\rStartAdvertisement");
  BuildStartAdvertisementMsg(100, sendMsg);
  r = AP_SendMessageResponse(sendMsg, RecvBuf, RECVSIZE);
  return r;
}
