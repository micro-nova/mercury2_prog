// Mercury 2 Programmer
// 
// Copyright (c) 2019 MicroNova LLC
// https://www.micro-nova.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
 
// ******************************************************************************
// *                              Includes                                      *
// ******************************************************************************

// Standard C libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

// OS-specific libraries
#ifdef _WIN32
  #include <windows.h>
#endif

// FTDI D2XX header
#include "ftd2xx.h"

// ******************************************************************************
// *                              Get time                                      *
// ******************************************************************************

// get timestamp in milliseconds
uint64_t get_timestamp_milliseconds(void)
{
  #ifdef _WIN32
    return clock();
  #else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t milliseconds = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
    return milliseconds;
  #endif
}

// ******************************************************************************
// *                              Sleep                                         *
// ******************************************************************************

// See: https://stackoverflow.com/a/28827188
void sleep_ms(int milliseconds) // cross-platform sleep function
{
#ifdef WIN32
    Sleep(milliseconds);
#elif _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
#else
    usleep(milliseconds * 1000);
#endif
}

// ******************************************************************************
// *                           FTDI global variables                            *
// ******************************************************************************

static FT_HANDLE FTDI_Handle;          // handle for FTDI channel
static FT_STATUS FTDI_Status = FT_OK;  // returned status

// number of FTDI devices found
#ifdef WIN32
  static long unsigned int FTDI_DeviceCount;     
#else
  static uint32_t  FTDI_DeviceCount;
#endif

// check ftStatus, exit app with error if failure occurred
#define FTDI_StatusCheck() {if(FTDI_Status!=FT_OK){printf("%s:%d:%s(): status(0x%x) \
!= FT_OK\n",__FILE__, __LINE__, __FUNCTION__,FTDI_Status);exit(1);}else{;}};

// ******************************************************************************
// *                            MPSSE pin setup commands                        *
// ******************************************************************************

// clock setup commands
const uint8_t MPSSE_SetClock[]   = {3, 0x8A, 0x97, 0x8D}; // use 60MHz clock, disable adaptive clock, disable 3 phase clock
const uint8_t MPSSE_SetDivisor[] = {3, 0x86, 0x00, 0x00}; // set clock divisor to 60MHz/(0+1)*2 = 30MHz
 
// ADBUS pins            │ DIRECTION │  STATE
// ──────────────────────┼───────────┼────────
// ADBUS[0] -> SPI SCLK  │  out  1   │  0
// ADBUS[1] -> SPI MOSI  │  out  1   │  0
// ADBUS[2] <- SPI MISO  │  in   0   │  0
// ADBUS[3] -> SPI CSN   │  out  1   │  0 = SPI flash asserted    1 = SPI flash idle
// ADBUS[4] -> n/c       │  out  1   │  0
// ADBUS[5] -> n/c       │  out  1   │  0         
// ADBUS[6] -> n/c       │  out  1   │  0
// ADBUS[7] -> n/c       │  out  1   │  0

#define ADBUS_GPIO_WRITE_COMMAND (0x80)
#define ADBUS_GPIO_READ_COMMAND  (0x81)
#define ADBUS_DIRECTION_IDLE     (0b00000000) // high-Z all
#define ADBUS_DIRECTION_ACTIVE   (0b11111011) // drive SCLK, MOSI, CSN and n/c pins
#define ADBUS_STATE_SPI_ASSERT   (0b00000000) // SPI asserted
#define ADBUS_STATE_SPI_IDLE     (0b00001000) // SPI idle

// ADBUS GPIO control commands
const uint8_t MPSSE_SpiAssert[]     = {3, ADBUS_GPIO_WRITE_COMMAND, ADBUS_STATE_SPI_ASSERT, ADBUS_DIRECTION_ACTIVE};
const uint8_t MPSSE_SpiIdle[]       = {3, ADBUS_GPIO_WRITE_COMMAND, ADBUS_STATE_SPI_IDLE,   ADBUS_DIRECTION_ACTIVE};
const uint8_t MPSSE_SpiDisconnect[] = {3, ADBUS_GPIO_WRITE_COMMAND, ADBUS_STATE_SPI_IDLE,   ADBUS_DIRECTION_IDLE};

// ACBUS pins            │ DIRECTION │  STATE
// ──────────────────────┼───────────┼─────────
// ACBUS[0] -> MUX SEL   │  out  1   │  0 = FTDI mux to SPI flash     1 = FTDI mux to FPGA JTAG (default via pull-up resistor)
// ACBUS[1] -> FPGA PROG │  out  1   │  0 = FPGA held in reset        1 = FPGA boots up (default via pull-up resistor)
// ACBUS[2] <- FPGA DONE │  in   0   │  0 = FPGA has not yet booted   1 = FPGA has booted successfully :)
// ACBUS[3] <- aux       │  in   0   │  0   
// ACBUS[4] <- aux       │  in   0   │  0   
// ACBUS[5] <- aux       │  in   0   │  0               
// ACBUS[6] <- aux       │  in   0   │  0   
// ACBUS[7] <- aux       │  in   0   │  0

#define ACBUS_GPIO_WRITE_COMMAND (0x82)
#define ACBUS_GPIO_READ_COMMAND  (0x83)
#define ACBUS_DIRECTION_IDLE     (0b00000000) // high-Z all
#define ACBUS_DIRECTION_ACTIVE   (0b00000011) // drive programming mux select and FPGA program
#define ACBUS_STATE_WRITE_FLASH  (0b00000000) // Mux to SPI flash, hold FPGA in reset
#define ACBUS_STATE_WRITE_JTAG   (0b00000011) // Mux to JTAG port, let FPGA boot

// ACBUS GPIO control commands
const uint8_t MPSSE_StateWriteFlash[]  = {3, ACBUS_GPIO_WRITE_COMMAND, ACBUS_STATE_WRITE_FLASH, ACBUS_DIRECTION_ACTIVE};
const uint8_t MPSSE_StateWriteJTAG[]   = {3, ACBUS_GPIO_WRITE_COMMAND, ACBUS_STATE_WRITE_JTAG,  ACBUS_DIRECTION_ACTIVE};
const uint8_t MPSSE_StateIdle[]        = {3, ACBUS_GPIO_WRITE_COMMAND, ACBUS_STATE_WRITE_JTAG,  ACBUS_DIRECTION_IDLE}; 

// ******************************************************************************
// *                             FTDI write buffer                              *
// ******************************************************************************

// FTDI write buffer
#define WRITE_BUFFER_MAX (512)
static uint8_t  FTDI_WriteBuffer[WRITE_BUFFER_MAX];  // buffer to be sent to FTDI
static uint32_t FTDI_WriteBuffer_Length = 0;         // length of data in buffer

// append byte to write buffer
void FTDI_WriteBuffer_AppendByte(uint8_t val){
  FTDI_WriteBuffer[FTDI_WriteBuffer_Length++]=val;
}

// append const array to write buffer
void FTDI_WriteBuffer_AppendArray(const uint8_t array[]){
  uint8_t LengthToSend = array[0];
  for(uint8_t i = 0; i < LengthToSend; i++){
    FTDI_WriteBuffer_AppendByte(array[i+1]);
  }
}

// clear write buffer
void FTDI_WriteBuffer_Reset(){
  FTDI_WriteBuffer_Length = 0;
}

// send out write buffer
void FTDI_WriteBuffer_Send(){
  #ifdef _WIN32
    long unsigned int BytesSent = 0;
  #else
    uint32_t BytesSent = 0;
  #endif
  FTDI_Status = FT_Write(FTDI_Handle, FTDI_WriteBuffer, FTDI_WriteBuffer_Length, &BytesSent); FTDI_StatusCheck();
  FTDI_WriteBuffer_Reset();
}

// print out write buffer
void FTDI_WriteBuffer_Debug(){
  for(int i = 0; i<FTDI_WriteBuffer_Length; i++)
  {
    printf("FTDI_WriteBuffer[%i] = 0x%02x\n", i, FTDI_WriteBuffer[i]);  
  }  
}

// ******************************************************************************
// *                              FTDI read buffer                              *
// ******************************************************************************

// FTDI read buffer
#define READ_BUFFER_MAX (512)
static uint8_t  FTDI_ReadBuffer[READ_BUFFER_MAX];

// FTDI read buffer length
#ifdef _WIN32
  static long unsigned int FTDI_ReadBuffer_Length = 0;
#else
  static uint32_t FTDI_ReadBuffer_Length = 0;
#endif

// How many bytes are in the FTDI read queue?
uint32_t FTDI_ReadQueueCount(){
  #ifdef _WIN32
    long unsigned int ReadQueueCount;
  #else
    uint32_t ReadQueueCount;
  #endif
  FTDI_Status = FT_GetQueueStatus(FTDI_Handle, &ReadQueueCount); FTDI_StatusCheck();
  return (uint32_t)ReadQueueCount;
}

// Process FTDI read queue, move data into application's read buffer
void FTDI_ReadQueueProcess(){
  FTDI_Status = FT_Read(FTDI_Handle, &FTDI_ReadBuffer, FTDI_ReadQueueCount(), &FTDI_ReadBuffer_Length);
  FTDI_StatusCheck();
}

// Wait until read queue has >= desired number of bytes
// TODO: add timeout!
void FTDI_ReadQueueWait(uint32_t desired_bytes)
{
  while(FTDI_ReadQueueCount() < desired_bytes);  
}

// ******************************************************************************
// *                         MPSSE SPI read/write commands                      *
// ******************************************************************************

// WRITE data bytes out, negative clock edge, MSB first (no read)
#define MPSSE_SPI_BYTES_OUT_NEG_MSB (0x11)

// READ data bytes in, positive clock edge, MSB first (no write)
#define MPSSE_SPI_BYTES_IN_POS_MSB  (0x20)

// macros to access high and low byte of 16-bit word
#define LOW_BYTE(x)  ((uint8_t)((x)&0xFF))
#define HIGH_BYTE(x) ((uint8_t)(((x)>>8)&0xFF))

// Add SPI write command to FTDI command buffer. 
// Follow with N bytes that you want to write.
void FTDI_WriteSPIBytesCommand(uint16_t num_bytes)
{
  FTDI_WriteBuffer_AppendByte(MPSSE_SPI_BYTES_OUT_NEG_MSB);    
  FTDI_WriteBuffer_AppendByte(LOW_BYTE(num_bytes-1));
  FTDI_WriteBuffer_AppendByte(HIGH_BYTE(num_bytes-1));    
}

// Add SPI read command to FTDI command buffer.
// After executing, FTDI will put the read bytes into the read buffer.
void FTDI_ReadSPIBytesCommand(uint16_t num_bytes)
{
  FTDI_WriteBuffer_AppendByte(MPSSE_SPI_BYTES_IN_POS_MSB);    
  FTDI_WriteBuffer_AppendByte(LOW_BYTE(num_bytes-1));
  FTDI_WriteBuffer_AppendByte(HIGH_BYTE(num_bytes-1));    
}

// ******************************************************************************
// *                         Read flash busy/idle status                        *
// ******************************************************************************

// read status register 1
#define SPI_READ_STATUS_REGISTER_1 (0x05)

// poll flash status
#define FLASH_IDLE (0)
#define FLASH_BUSY (1)
uint8_t Flash_GetStatus(void)
{
  // prepare FTDI commands
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);           // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(1);                            // Send 1 SPI byte
  FTDI_WriteBuffer_AppendByte(SPI_READ_STATUS_REGISTER_1); // SPI command - read status register
  FTDI_ReadSPIBytesCommand(1);                             // Read 1 SPI byte
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);             // Deassert SPI chip select
  
  // send out
  FTDI_WriteBuffer_Send(); // send FTDI commands
  FTDI_ReadQueueWait(1);   // wait for 1 byte to be available in the RX queue
  FTDI_ReadQueueProcess(); // process read queue and print

  // return status bit
  return (FTDI_ReadBuffer[0] & 0x01);
}

// ******************************************************************************
// *                          Flash identification                              *
// ******************************************************************************

// capacities
#define FLASH_UNKNOWN   (0x00)
#define FLASH_S25FL116K (0x15)
#define FLASH_S25FL132K (0x16)
#define FLASH_S25FL164K (0x17)


// Read flash manufacturer ID, device type, and capacity.
// Return flash capacity in bytes.
int32_t Flash_ReadCapacity(void)
{
  // flash size in bytes
  int32_t Flash_CapacityBytes = 0;  
  // assert CS
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);
  // send flash read command
  FTDI_WriteBuffer_AppendByte(0x11);
  FTDI_WriteBuffer_AppendByte(0x00);    
  FTDI_WriteBuffer_AppendByte(0x00);
  FTDI_WriteBuffer_AppendByte(0x9F);
  // read 3 bytes
  FTDI_WriteBuffer_AppendByte(0x20);
  FTDI_WriteBuffer_AppendByte(0x02);    
  FTDI_WriteBuffer_AppendByte(0x00); 
  // deassert CS
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);
  FTDI_WriteBuffer_Send();  
  
  // wait a bit
  sleep_ms(10);
  
  // At this point, there should be 3 bytes in the queue:
  // FTDI_ReadBuffer[0] = manufacturer ID (0x01)
  // FTDI_ReadBuffer[1] = device ID (0x40)
  // FTDI_ReadBuffer[2] = flash capacity (0x15, 0x16, 0x17, etc)
  if(FTDI_ReadQueueCount() == 3)
  {
    // process read queue
    FTDI_ReadQueueProcess();
    // check flash manufacturer and device ID
    if(FTDI_ReadBuffer[0] == 0x01 && FTDI_ReadBuffer[1] == 0x40)
    {
      // check flash capacity
      switch(FTDI_ReadBuffer[2])
      {
        case 0x15: printf("Found flash: S25FL116K 16Mbit\n"); Flash_CapacityBytes = 4096*512;  break;
        case 0x16: printf("Found flash: S25FL132K 32Mbit\n"); Flash_CapacityBytes = 4096*1024; break;
        case 0x17: printf("Found flash: S25FL164K 64Mbit\n"); Flash_CapacityBytes = 4096*2048; break;
        default:   printf("ERROR: Unknown flash capacity (%x)\n", FTDI_ReadBuffer[2]); exit(1);  break;
      }        
    }
    else
    {
      // print debug information
      printf("ERROR: Unknown flash manufacturer and/or device ID\n");
      printf("Manufacturer = 0x%02X (expected 0x01)\n", FTDI_ReadBuffer[0]);
      printf("Device ID    = 0x%02X (expected 0x40)\n", FTDI_ReadBuffer[1]);      
      printf("Capactiy     = 0x%02X (expected 0x15, 0x16, 0x17)\n", FTDI_ReadBuffer[2]);      
      exit(1);
    }
  }
  else
  {
    // print debug information
    printf("ERROR: There are %i bytes in the queue. Expected 3 bytes.\n",FTDI_ReadQueueCount());
    FTDI_ReadQueueProcess();
    for(int i = 0; i<FTDI_ReadBuffer_Length; i++)
    {
      printf("FTDI_ReadBuffer[%i] = 0x%02X\n", i, FTDI_ReadBuffer[i]);
    }  
    exit(1);
  }
  
  // return flash bytes
  return Flash_CapacityBytes;
}

/******************************************************************************/
/*                            Flash erasure                                   */
/******************************************************************************/

// Erase one block (64kB)
void Flash_EraseBlock(uint8_t block_addr, uint8_t n_blocks_total)
{
  // WRITE ENABLE COMMAND
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);   // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(1);                    // Send 1 SPI byte
  FTDI_WriteBuffer_AppendByte(0x06);               // FLASH COMMAND: write enable
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);     // Deassert SPI chip select  

  // BLOCK ERASE COMMAND
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);   // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(4);                    // Send 4 SPI bytes
  FTDI_WriteBuffer_AppendByte(0xD8);               // FLASH COMMAND: block erase commands
  FTDI_WriteBuffer_AppendByte(block_addr);         // 24 downto 16 
  FTDI_WriteBuffer_AppendByte(0x00);               // 15 downto 8 
  FTDI_WriteBuffer_AppendByte(0x00);               // 7 downto 0  
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);     // Deassert SPI chip select
  FTDI_WriteBuffer_Send();                         // send out

  printf("Erasing block %i / %i = %.1f%%   \r", block_addr, n_blocks_total-1, ((float)(block_addr+1) / (float)n_blocks_total) * 100.0); 
  fflush(stdout);
  while(Flash_GetStatus() == FLASH_BUSY);
}

// erase first N blocks
void Flash_EraseBlocks(uint32_t n_blocks)
{
  for(uint32_t i=0;i<n_blocks;i++)
  { 
    Flash_EraseBlock(i, n_blocks);
  }
  printf("\n");
}

/******************************************************************************/
/*                             Flash read                                     */
/******************************************************************************/

uint8_t Flash_ReadPage(uint32_t page_addr, uint8_t* page_data, uint32_t pages_total)
{
  uint32_t page_address_upper = (0xFF & (page_addr >> 8));
  uint32_t page_address_lower = (0xFF & (page_addr >> 0));

  printf("Reading page %i / %i = %.1f%%   \r", page_addr, pages_total-1, ((float)(page_addr+1) / (float)pages_total) * 100.0); 
  
  // assert CS, read command
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);   // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(4);                    // Send 4 SPI bytes  
  FTDI_WriteBuffer_AppendByte(0x03);               // FLASH COMMAND: read data
  FTDI_WriteBuffer_AppendByte((uint8_t)page_address_upper); // 24 downto 16
  FTDI_WriteBuffer_AppendByte((uint8_t)page_address_lower); // 15 downto 8
  FTDI_WriteBuffer_AppendByte(0x00);                        // 7 downto 0
  
   // read 256 bytes
  FTDI_WriteBuffer_AppendByte(0x20);
  FTDI_WriteBuffer_AppendByte(0xFF);    
  FTDI_WriteBuffer_AppendByte(0x00);
  
  // deassert CS, send to FTDI
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);
  FTDI_WriteBuffer_Send();  

  // wait until bytes are read
  // TODO: Add a timeout here
  while(FTDI_ReadQueueCount() < 256);
  
  // process queue
  FTDI_ReadQueueProcess();
  
  // copy to page data buffer
  memcpy(page_data, FTDI_ReadBuffer, 256); 
}


/******************************************************************************/
/*                            Flash write                                     */
/******************************************************************************/

// Write one page of flash (256B)
void Flash_WritePage(uint32_t page_addr, uint8_t* bitstream, uint32_t pages_to_write)
{
  uint32_t page_address_upper = (0xFF & (page_addr >> 8));
  uint32_t page_address_lower = (0xFF & (page_addr >> 0));
  
  // WRITE ENABLE COMMAND
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);   // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(1);                    // Send 1 SPI byte
  FTDI_WriteBuffer_AppendByte(0x06);               // FLASH COMMAND: write enable
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);     // Deassert SPI chip select  

  // WRITE PAGE COMMAND
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiAssert);   // Assert SPI chip select
  FTDI_WriteSPIBytesCommand(260);                  // We are writing 1 byte command + 3 bytes address + 256 bytes data
  FTDI_WriteBuffer_AppendByte(0x02);               // FLASH COMMAND: page program
  FTDI_WriteBuffer_AppendByte((uint8_t)page_address_upper); // 24 downto 16
  FTDI_WriteBuffer_AppendByte((uint8_t)page_address_lower); // 15 downto 8
  FTDI_WriteBuffer_AppendByte(0x00);                        // 7 downto 0      

  // WRITE PAGE DATA
  memcpy(FTDI_WriteBuffer+FTDI_WriteBuffer_Length, bitstream+(256*page_addr), 256); // append bitstream data to FTDI buffer
  FTDI_WriteBuffer_Length+=256; 
  
  // SEND OUT
  FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);     // Deassert SPI chip select 
  FTDI_WriteBuffer_Send();                         // send out  

  // Wait for completion
  printf("Writing page %i / %i = %.1f%%   \r", page_addr, pages_to_write-1, ((float)(page_addr+1) / (float)pages_to_write) * 100.0);   
  while(Flash_GetStatus() == FLASH_BUSY);
}

// ******************************************************************************
// *                   FTDI connection and initialization                       *
// ******************************************************************************

// print out FTDI device type
void FTDI_PrintDeviceType(int device_type)
{
 switch(device_type)
 {
   case FT_DEVICE_BM:    printf("FT232BM"); break;
   case FT_DEVICE_AM:    printf("FT232AM"); break;
   case FT_DEVICE_2232C: printf("FT2232C"); break;
   case FT_DEVICE_232R:  printf("FT232R");  break;
   case FT_DEVICE_2232H: printf("FT2232H"); break;
   case FT_DEVICE_4232H: printf("FT4232H"); break;
   case FT_DEVICE_232H:  printf("FT232H");  break;
   default: printf("Unknown (0x%02X)", device_type); break;
 } 
}

// Print FTDI device list and return index of first Mercury 2 board
// TODO: Update app to allow users to select from multiple connected Mercury 2 FPGAs
int FTDI_FindMercury2(void)
{
  // Build FTDI device information list
  printf("Checking for FTDI devices...");
  FTDI_Status = FT_CreateDeviceInfoList(&FTDI_DeviceCount); FTDI_StatusCheck();
  
  // Print list of FTDI devices
  printf("%d FTDI devices found\n", FTDI_DeviceCount);
  if(FTDI_DeviceCount > 0){
    
    // Fill FTDI device list
    FT_DEVICE_LIST_INFO_NODE *devInfo;
    devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*FTDI_DeviceCount);
    FTDI_Status = FT_GetDeviceInfoList(devInfo, &FTDI_DeviceCount);
    if(FTDI_Status == FT_OK){
      
      // Print FTDI device list
      for (int i = 0; i < FTDI_DeviceCount; i++) {
        printf("Device %d:\n",i);
        printf("  - Description = '%s'\n", devInfo[i].Description);
        printf("  - Serial #    = '%s'\n",   devInfo[i].SerialNumber);
        printf("  - FTDI Chip   = "); FTDI_PrintDeviceType(devInfo[i].Type); printf(" @ "); 
        printf("%s\n", (devInfo[i].Flags & FT_FLAGS_HISPEED) ? "480Mbps" : "12Mbps");
        printf("  - Opened      = %s\n", (devInfo[i].Flags & FT_FLAGS_OPENED) ? "Yes" : "No");        
      }
      printf("\n");
      
      // Find channel A (programming channel) of first Mercury 2 FPGA
      for (int i = 0; i < FTDI_DeviceCount; i++) {
        if(!strcmp(devInfo[i].Description, "Mercury 2 FPGA A") && !(devInfo[i].Flags & FT_FLAGS_OPENED))
        {
          // Mercury 2 found
          printf("Found Mercury 2 FPGA board at FTDI device %i.\n", i); 
          return i;  
        }
      }
    }
    else
    {
      printf("ERROR: Unable to enumerate FTDI devices.\n"); 
      exit(1);      
    }
  }
  // Mercury 2 not found
  printf("ERROR: No Mercury 2 FPGA board found.\n"); 

  #ifndef _WIN32
  printf("\n");
  printf("Please note that you may have to disable the FTDI VCP driver in order to use the D2XX driver.\n");
  printf("You can do this by running: \033[0;1msudo rmmod ftdi_sio && sudo rmmod usbserial\033[0m\n");
  printf("\n");
  printf("See FTDI Application Note AN_220 for more details:\n");
  printf("https://www.ftdichip.com/Support/Documents/AppNotes/AN_220_FTDI_Drivers_Installation_Guide_for_Linux.pdf\n");
  printf("\n");
  #endif
  exit(1);
}

// Attempt to connect to Mercury 2 board with specified FTDI device ID
void FTDI_ConnectMercury2(int deviceID)
{
    // open FTDI device
    FTDI_Status = FT_Open(deviceID,&FTDI_Handle);
    if (FTDI_Status == FT_OK) {
      printf("FTDI device %i opened successfully.\n", deviceID);
    }
    else {
      printf("ERROR: Unable to open FTDI device %i.\n", deviceID);
      exit(1);
    }  
  
    // configuration (see FT_000208, section 4.2 and section 5.2)
    printf("Configuring FTDI for SPI communication...");
    FTDI_Status |= FT_ResetDevice(FTDI_Handle);                           FTDI_StatusCheck();  // reset device
    
    // this doesn't seem to work on Windows...
    #ifndef _WIN32
      FTDI_Status |= FT_SetUSBParameters(FTDI_Handle, 64, 0);             FTDI_StatusCheck();  // set input transfer size to 64 bytes
    #endif
      
    FTDI_Status |= FT_SetChars(FTDI_Handle, 0, 0, 0, 0);                  FTDI_StatusCheck();  // set error characters to none
    FTDI_Status |= FT_SetLatencyTimer(FTDI_Handle, 0);                    FTDI_StatusCheck();  // set latency timer to 2ms
    FTDI_Status |= FT_SetFlowControl(FTDI_Handle, FT_FLOW_RTS_CTS, 0, 0); FTDI_StatusCheck();  // set flow control to RTS/CTS
    FTDI_Status |= FT_SetBitMode(FTDI_Handle, 0x00, 0x00);                FTDI_StatusCheck();  // set bit mode = 0 (RESET)
    FTDI_Status |= FT_SetBitMode(FTDI_Handle, 0x00, 0x02);                FTDI_StatusCheck();  // set bit mode = 2 (MPSSE)
    FTDI_Status |= FT_Purge(FTDI_Handle, FT_PURGE_RX | FT_PURGE_TX);      FTDI_StatusCheck();  // purge RX and TX queues
    
    // configure MPSSE clock
    FTDI_WriteBuffer_AppendArray(MPSSE_SetClock);
    FTDI_WriteBuffer_AppendArray(MPSSE_SetDivisor);
    FTDI_WriteBuffer_Send();

    // initialize pins
    FTDI_WriteBuffer_AppendArray(MPSSE_StateWriteFlash); // mux to flash, hold FPGA in reset
    FTDI_WriteBuffer_AppendArray(MPSSE_SpiIdle);         // set SPI lines idle
    FTDI_WriteBuffer_Send();
    
    printf("OK\n");
}

// release SPI lines and let FPGA boot
void FTDI_ReleaseMercury2(void)
{
    FTDI_WriteBuffer_AppendArray(MPSSE_SpiDisconnect);
    FTDI_WriteBuffer_AppendArray(MPSSE_StateIdle);
    FTDI_WriteBuffer_Send();
}

// ******************************************************************************
// *                            Dump flash to file                              *
// ******************************************************************************

void Flash_DumpToFile(int PagesToDump)
{
  char* dump_filename = "dump.bit";
  FILE *dump_fptr;
  int dump_write_status;
  printf("\n***** DUMPING FLASH *****\n");
  printf("Saving %i MBytes (%i Mbits) to '%s'\n", PagesToDump/4096, PagesToDump/512, dump_filename);

  // try to open file
  dump_fptr = fopen(dump_filename, "wb");
  if(dump_fptr == NULL || dump_fptr == 0)
  {
    printf("ERROR: Unable to open '%s'\n", dump_filename);
    exit(1);
  }  

  // allocate a 256 byte RAM chunk for page dump
  uint8_t* dump;
  dump = (uint8_t *)malloc(256 * sizeof(uint8_t));
  if(dump == 0)
  {
    printf("ERROR: not enough RAM!\n");
    exit(1);
  }  
  
  // read from flash, write to file
  for(int i=0; i<PagesToDump; i++)
  {
    Flash_ReadPage(i, dump, PagesToDump); // read page into RAM
    if(fwrite(dump, sizeof(uint8_t), 256, dump_fptr) == 0) // write page to file
      printf("ERROR: Unable to write to '%s'\n", dump_filename);
  }
  printf("\n");
  
  // close file
  fclose(dump_fptr);
}

// ******************************************************************************
// *                              Main function                                 *
// ******************************************************************************

int main(int argc, char** argv)
{
  // welcome message
  printf(".----------------------------------.\n");  
  printf("|       MERCURY 2 PROGRAMMER       |\n");
  printf("|      (c) 2019 MicroNova LLC      |\n");
  printf("|        www.micro-nova.com        |\n");
  printf("'----------------------------------'\n");  
  printf("\n");
  
  // command line arguments
  if(argc<=1){
    printf("ERROR: No command line arguments given. Try '-h' for help.\n"); exit(1);
  }else{
    // *@@@@@@@@@@@@@@@@@@ LIST FTDI DEVICES @@@@@@@@@@@@@@@@@@@@
    if(!strcmp(argv[1], "-l") || !strcmp(argv[1], "--list"))
    {
      FTDI_FindMercury2();
    }
    
    // *@@@@@@@@@@@@@@@@@@@@@@@ HELP @@@@@@@@@@@@@@@@@@@@@@@@@@@@
    else if(!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))
    {
      printf("OPTION            DESCRIPTION\n");
      printf("-h, --help        print this help message\n");
      printf("-l, --list        list all visible FTDI devices\n");
      printf("-e, --erase       erase entire flash \n");
      printf("-r, --read        read entire flash and save to 'dump.bit' \n");
      printf("-w, --write       write flash using specified file\n");
      printf("\n");
    }
    
    // *@@@@@@@@@@@@@@@@@@@@@@@ ERASE @@@@@@@@@@@@@@@@@@@@@@@@@@@
    else if(!strcmp(argv[1], "-e") || !strcmp(argv[1], "--erase"))
    {
      FTDI_ConnectMercury2(FTDI_FindMercury2());     // connect to Mercury 2
      printf("\n***** ERASING FLASH *****\n");
      Flash_EraseBlocks(Flash_ReadCapacity()/65535); // erase flash
      FTDI_ReleaseMercury2();                        // release SPI lines and let FPGA boot
    }    
    
    // *@@@@@@@@@@@@@@@@@@@@@ READ FLASH @@@@@@@@@@@@@@@@@@@@@@@@
    else if(!strcmp(argv[1], "-r") || !strcmp(argv[1], "--read"))
    {
      FTDI_ConnectMercury2(FTDI_FindMercury2());  // connect to Mercury 2
      Flash_DumpToFile(Flash_ReadCapacity()/256); // dump flash to file
      FTDI_ReleaseMercury2();                     // release SPI lines and let FPGA boot
    }
    
    // @@@@@@@@@@@@@@@@@@@@@ WRITE FLASH @@@@@@@@@@@@@@@@@@@@@@@@
    else if(!strcmp(argv[1], "-w") || !strcmp(argv[1], "--write"))
    {
      // check if user specified a parameter for filename
      if(argc<3){
        printf("ERROR: Please specify a filename.\n"); 
        exit(1);
      }

      // attempt to open file
      FILE *fptr;
      char* filename = argv[2];
      if ((fptr = fopen(filename, "rb")) == NULL) // "rb" = open for reading, binary mode
      {
        printf("ERROR: Could not open file '%s'\n", filename); 
        exit(1);
      }   
      
      // get file bytes
      printf("Reading bitstream file '%s'\n\n", filename);
      fseek(fptr, 0, SEEK_END);      // set file position to end of file
      int file_bytes = ftell(fptr);  // get index of current file position
      fseek(fptr, 0, SEEK_SET);      // set file position to beginning of file
      
      // calculate pages (256 bytes)    
      int file_pages = file_bytes / 256;                                            // get pages
      int file_pages_rem_bytes = file_bytes - (file_pages * 256);                   // get remainder in bytes
      int pages_to_write = file_pages + ((file_pages_rem_bytes > 0) ? 1 : 0);       // 1 more page for remainder

      // calculate sectors (4096 bytes)
      int file_sectors = file_bytes / 4096;                                         // get sectors
      int file_sectors_rem_bytes = file_bytes - (file_sectors * 4096);              // get remainder in bytes
      int sectors_to_write = file_sectors + ((file_sectors_rem_bytes > 0) ? 1 : 0); // 1 more sector for remainder   

      // calculate blocks (64 kBytes)
      int file_blocks = file_bytes / 65535;                                         // get blocks
      int file_blocks_rem_bytes = file_bytes - (file_blocks * 65535);               // get remainder in bytes
      int blocks_to_write = file_blocks + ((file_blocks_rem_bytes > 0) ? 1 : 0);    // 1 more block for remainder      
      
      // print size information
      printf("Size is %.1f kBytes = %d bytes = %d bits\n",(float)(file_bytes)/1024.0,file_bytes,file_bytes*8);
      printf("%5d x 256B pages   (%d whole pages + %d byte remainder)\n",pages_to_write, file_pages, file_pages_rem_bytes);
      printf("%5d x 4kB  sectors (%d whole sectors + %d byte remainder)\n",sectors_to_write, file_sectors, file_sectors_rem_bytes);
      printf("%5d x 64kB blocks  (%d whole blocks + %d byte remainder)\n",blocks_to_write, file_blocks, file_blocks_rem_bytes);
      printf("\n");
      
      // allocate chunk of RAM for bitstream
      uint8_t *bitstream;
      bitstream = (uint8_t *)calloc(pages_to_write * 256, sizeof(uint8_t));
      if(bitstream == 0)
      {
        printf("ERROR: not enough RAM!\n"); fflush(stdout);
        exit(1);
      }
      
      // read entire bitstream file into RAM, close file
      printf("Reading file...\n");
      if(fread(bitstream, 1, file_bytes, fptr) == 0)
      {
        printf("ERROR: problem reading bitstream file.\n"); fflush(stdout);
        exit(1);
      }
      fclose(fptr);
      
      // check for valid Xilinx bitstream header
      const uint8_t expected_header[14] = {0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x00, 0x00, 0x01, 0x61};
      uint8_t header_ok = 1;
      for(int i=0; i<sizeof(expected_header); i++)
      {
        if(bitstream[i] != expected_header[i])
          header_ok = 0;
      }
      
      // return error if bitstream header is incorrect
      if(header_ok == 0){
        // print what we read
        printf("    Read header: ");
        for(int i=0; i<sizeof(expected_header); i++)
          printf("%02X ", bitstream[i]);
        printf("\n");
        // print what we expected
        printf("Expected header: ");
        for(int i=0; i<sizeof(expected_header); i++)
          printf("%02X ", expected_header[i]);
        printf("\n");
        // error!
        //printf("\nERROR: Incorrect header. Is this a Xilinx bitstream file?\n");
        //exit(1);
	printf("\nWARNING: Incorrect header. Is this a Xilinx bitstream file?\n");
      }
      
      // connect to Mercury 2 
      FTDI_ConnectMercury2(FTDI_FindMercury2());
      
      // make sure flash is big enough
      int flash_bytes = Flash_ReadCapacity();
      if(file_bytes > flash_bytes)
      {
        printf("\nERROR: Flash is too small. (%i bytes required / %i bytes available)\n", file_bytes, flash_bytes);
        exit(1);
      }
      
      // total time - START
      uint64_t total_start = get_timestamp_milliseconds();

      // flash necessary pages
      printf("\n***** WRITING FLASH *****\n");
      // erase blocks
      Flash_EraseBlocks(blocks_to_write);
      // write pages
      for(int j=0;j<pages_to_write;j++)
      {
        Flash_WritePage(j, bitstream, pages_to_write);
      }
    
      // total time - STOP
      uint64_t total_stop = get_timestamp_milliseconds();
      float total_time = (float)(total_stop-total_start) / 1000;
      
      // output statistics
      printf("\nProgramming time : %.3f sec\n", total_time);
      printf("Effective speed  : %.3f kBytes/sec\n", (float)(file_bytes/1024) / total_time);
    
      // release SPI lines and let FPGA boot
      FTDI_ReleaseMercury2();
      
      sleep_ms(100);
    }
    
    // @@@@@@@@@@@@@@@@@@@@ UNRECOGNIZED OPTION @@@@@@@@@@@@@@@@@@@@
    else
    {
      printf("Unrecognized option. Try '-h' for help.\n");
      exit(1);
    }
  }
  
  // exit without errors
  return(0);  
}

