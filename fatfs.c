/*
//example used in debugging/testing
            strcpy(BSI_Attribute.BSI_Name, "BSI_TEST.txt");
            for(uint8_t inc; inc < 13; inc++)
            {
              BSI_Data.SensorValue = 0X0ff1;
              BSI_Data.CountMin = inc*20;       
              fatfs_write(&BSI_Data);
            }
This file contains the functions to pull data from input data stream, format it and store it to an SD card
Currently the variable that must be written to are:
-BSI_Attribute.BSI_Name
-BSI_Attribute.StartTime
-BSI_Data.SensorValue
-BSI_Data.CountMin
*/
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "fatfs.h"


#define FILE_NAME   "NORDIC.TXT"
#define TEST_STRING "\nHI"
#define SDC_BUFFER_SIZE 512

#define SDC_SCK_PIN     NRF_GPIO_PIN_MAP(1,9)  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    NRF_GPIO_PIN_MAP(1,8)  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    NRF_GPIO_PIN_MAP(0,4)  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      NRF_GPIO_PIN_MAP(0,26)  ///< SDC chip select (CS) pin.

#define DEBUG

static FIL file;

//redefine the extern struct in fatfs.h
BSI_Att_t BSI_Attribute = {
          .BSI_Name = {},
          .Start_Time = {},
          };
BSI_Data_t BSI_Data = {};

NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

// Initialize FATFS disk I/O interface by providing the block device.
static diskio_blkdev_t drives[] =
{
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
};

void data_ch_decode(BSI_Data_t * tempValue)
{
  tempValue->SensorCh = tempValue->SensorValue & 0x000F;
  tempValue->SensorValue = (tempValue->SensorValue >> 4);
}

//function to calculate the time that a measurement occured based on the minutes received
void current_time(uint16_t minutes, char *CurrentTimeString, uint8_t *CurrentTime)
{
    uint8_t days_in_month, days, hours, mins;
    char TempStr[2];

    days = minutes/1440;            //this will ensure that time added to array is not greater the one increment for rollover
    hours = (minutes - (1440*days))/60;
    mins = (minutes - (1440*days) - (60*hours));


    //uint8_t StartTime[8]; //7 bytes (5+ 2byte year) YY/YY/MM/DD/HH/MM/SS
    //                                               [0][1][2][3][4][5][6]

    CurrentTime[5] += mins;
    CurrentTime[4] += hours;
    CurrentTime[3] += days;
    
    if(CurrentTime[5] >= 60) //rollover minutes & increment hours
    {
      CurrentTime[4]++;
      CurrentTime[5] %= 60;  //assign the remainder to the array
    }
    if(CurrentTime[4] >= 24) //rollover hours & increment days
    {
      CurrentTime[3]++;
      CurrentTime[4] %= 24;  //assign the remainder to the array
    }
    if(CurrentTime[2] == 1 ||
         CurrentTime[2] == 3 ||
         CurrentTime[2] == 5 ||
         CurrentTime[2] == 7 ||
         CurrentTime[2] == 8 ||
         CurrentTime[2] == 10 ||
         CurrentTime[2] == 12) //if the month has 31 days
    {
      days_in_month = 31;
    }
    else if(CurrentTime[2] == 2) //if it is feb
    {
      if((CurrentTime[1] % 4) == 0) //if it is a leap year
      {
        days_in_month = 29;
      }
      else
      {
        days_in_month = 28;
      }
    }
    else
    {
      days_in_month = 30;
    }
    if(CurrentTime[3] > days_in_month) //rollover days & increment months
    {
      
      CurrentTime[2]++;
      CurrentTime[3] %= days_in_month;  //assign the remainder to the array
    }
    if(CurrentTime[2] > 12) //rollover months & increment year
    {
      CurrentTime[1]++;
      CurrentTime[2] = 1;  //reset months
    }
    if(CurrentTime[1] > 99) //rollover year & increment centuries
    {
      
      CurrentTime[0]++;
      CurrentTime[1] = 0;  //reset
    }

    for(uint8_t i = 0; i <= 5; i++) //for loop will contruct a char array of the time-date format from the int array passed
    {
      utoa(CurrentTime[i], TempStr, 10);
      //uint8_t StartTime[8]; //7 bytes (5+ 2byte year) YY/YY/MM/DD/HH/MM/SS
      //                                               [0][1][2][3][4][5][6]
      if(CurrentTime[i] < 10)
      {
        TempStr[1] = TempStr[0];
        TempStr[0] = '0';
      }
      if(i >= 2 && i <= 4)
      {
        strncat(CurrentTimeString, "/", 1);
      }
      else if (i > 4)
      {
        strncat(CurrentTimeString, ":", 1);
      }
      strncat(CurrentTimeString, TempStr, 2);
    }
}

void fatfs_write(fatfs_write_buffer_t fatfs_buffer)
{
    uint32_t bytes_written;
    FRESULT ff_result;

    #ifdef DEBUG
    printf("Opening File BSI_Name... \n");
    #endif

    ff_result = f_open(&file, BSI_Attribute.BSI_Name, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
      #ifdef DEBUG
      printf("Unable to open or create file: BSI_Name.\n");  
      #endif
    }
    
    printf("Writing to file BSI_Name...\n");
    ff_result = f_write(&file, fatfs_buffer.data, fatfs_buffer.length, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
      #ifdef DEBUG
      printf("Write failed\r\n.");
      #endif
    }
   else
   {
      #ifdef DEBUG
      printf("%d bytes written.\n", bytes_written);
      #endif
      fatfs_buffer.length = 0;
   }

   (void) f_close(&file);
}

void fatfs_bsi_data_write(uint8_t *rx_data_8bit, uint16_t rx_length, bool first_rx)//need to pass array and length
{
    uint16_t rx_data_16bit[rx_length];
    //bool first_rx;         //for testing this may need to be global for final implementation
    char SensorValStr[6];
    uint8_t CurrentTimeTemp[8];
    char CurrentTimeStr[20];
    char TempStr[2];
    fatfs_write_buffer_t fatfs_write_buffer = {}; 

    //16 bytes for friendly name
    //8 bytes for time
    //only run on initial call 
    //for debugging
    #ifdef DEBUG
    first_rx = true;  
    #endif

    if(first_rx)
    {
      memcpy(BSI_Attribute.BSI_Name, rx_data_8bit, 16);
      //strncat(BSI_Attribute.BSI_Name, &rx_data_8bit[0], 16);  //first 16 bits are reserved for the friendly name
      strcat(BSI_Attribute.BSI_Name, ".txt");
      memmove(rx_data_8bit, &rx_data_8bit[16], (rx_length - 16));      //shift the array by 16 
      strncat(BSI_Attribute.Start_Time, rx_data_8bit, 6); //next 6 bits are reserved for the time
      memmove(rx_data_8bit, &rx_data_8bit[6], (rx_length - 22));       //shift the array by 6
      memset(&rx_data_8bit[rx_length-22], 0, 22);                  //set last 22 values of array to null as they are no longer valid. 
      rx_length = rx_length - 22;                                      //adjust length the proper value
    }
    
    //for debugging
    #ifdef DEBUG
    first_rx = false;
    #endif

    memset(fatfs_write_buffer.data, 0, SDC_BUFFER_SIZE);      //clear write buffer
    //memcpy(rx_data_16bit, rx_data_8bit, rx_length);         //convert data received to 16 bit big endian
    for(uint16_t count = 0; count <= rx_length; count += 2)   //convert data received to 16 bit lil-endian
    {
      rx_data_16bit[count/2] = (rx_data_8bit[count] << 8) + rx_data_8bit[count+1];
    }

    for(uint16_t inc = 0; inc < (rx_length/2); inc += 2)
    {
      //fatfs_example();
      BSI_Data.CountMin = rx_data_16bit[inc];
      BSI_Data.SensorValue = rx_data_16bit[inc+1];
      data_ch_decode(&BSI_Data);
      memset(CurrentTimeStr, 0, 20);
      strcpy(CurrentTimeTemp, BSI_Attribute.Start_Time);
      current_time(BSI_Data.CountMin, CurrentTimeStr, CurrentTimeTemp);

      utoa(BSI_Data.SensorValue, SensorValStr, 10); //convert sensor data to buffer data
      utoa(BSI_Data.SensorCh, TempStr, 10);         //convert sensor channel to buffer data
      
       //convert format to [time,ch,data]
      strcat(fatfs_write_buffer.data, CurrentTimeStr);
      strcat(fatfs_write_buffer.data, ",");
      strcat(fatfs_write_buffer.data, TempStr);
      strcat(fatfs_write_buffer.data, ",");
      strncat(fatfs_write_buffer.data, SensorValStr, 4);
      strcat(fatfs_write_buffer.data, "\n");              //format time, channel, data \n
      
      fatfs_write_buffer.length = strlen(fatfs_write_buffer.data);  
      #ifdef DEBUG
      printf("%d \n", inc);
      #endif
      if(fatfs_write_buffer.length >= (SDC_BUFFER_SIZE - 25)) //if buffer is full, write data
      {   
          //fatfs_write_buffer.length = strlen(fatfs_write_buffer.data);        //update length
          fatfs_write(fatfs_write_buffer);                                    //write data
          memset(fatfs_write_buffer.data, 0, SDC_BUFFER_SIZE);                            //clear write buffer   
          fatfs_write_buffer.length = 0;                                      //clear length
      }

    }
    fatfs_write_buffer.length = strlen(fatfs_write_buffer.data);              //write whatever is left of the buffer
    fatfs_write(fatfs_write_buffer);

//test header
//    uint8_t byte_array_hex[254] = {'B', 'S', 'I', 'T', 'E', 'S', 'T', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                  20, 19, 11, 18, 11, 22, 
//                                  0x0, 0x1, 0x0f, 0xf1, 
//                                  0x0, 0x2, 0x1c, 0x11, 
//                                  0x0, 0x3, 0x20, 0x22, 
//                                  0x0, 0x4, 0x24, 0x21, 
//                                  0x0, 0x5, 0x28, 0x22, 
//                                  0x0, 0x6, 0x2c, 0x21, 
//                                  0x0, 0x7, 0x30, 0x32, 
//                                  0x0, 0x8, 0x34, 0x31, 
//                                  0x0, 0x9, 0x38, 0x32, 
//                                  0x0, 0xa, 0x3c, 0x31, 
//                                  0x0, 0xb, 0x40, 0x42, 
//                                  0x0, 0xc, 0x44, 0x41, 
//                                  0x0, 0xd, 0x48, 0x42, 
//                                  0x0, 0xe, 0x4c, 0x41, 
//                                  0x0, 0xf, 0x50, 0x52, 
//                                  0x0, 0x10, 0x54, 0x51, 
//                                  0x0, 0x11, 0x58, 0x52, 
//                                  0x0, 0x12, 0x5c, 0x51, 
//                                  0x0, 0x13, 0x60, 0x62, 
//                                  0x0, 0x14, 0x64, 0x61, 
//                                  0x0, 0x15, 0x68, 0x62, 
//                                  0x0, 0x16, 0x6c, 0x61, 
//                                  0x0, 0x17, 0x70, 0x72, 
//                                  0x0, 0x18, 0x74, 0x71, 
//                                  0x0, 0x19, 0x78, 0x72, 
//                                  0x0, 0x1a, 0x7c, 0x71, 
//                                  0x0, 0x1b, 0x80, 0x82, 
//                                  0x0, 0x1c, 0x84, 0x81, 
//                                  0x0, 0x1d, 0x88, 0x82, 
//                                  0x0, 0x1e, 0x8c, 0x81, 
//                                  0x0, 0x1f, 0x90, 0x92, 
//                                  0x0, 0x20, 0x94, 0x91, 
//                                  0x0, 0x21, 0x98, 0x92, 
//                                  0x0, 0x22, 0x9c, 0x91, 
//                                  0x0, 0x23, 0xa0, 0xa2, 
//                                  0x0, 0x24, 0xa4, 0xa1, 
//                                  0x0, 0x25, 0xa8, 0xa2, 
//                                  0x0, 0x26, 0xac, 0xa1, 
//                                  0x0, 0x27, 0xb0, 0xb2, 
//                                  0x0, 0x28, 0xb4, 0xb1, 
//                                  0x0, 0x29, 0xb8, 0xb2, 
//                                  0x0, 0x2a, 0xbc, 0xb1, 
//                                  0x0, 0x2b, 0xc0, 0xc2, 
//                                  0x0, 0x2c, 0xc4, 0xc1, 
//                                  0x0, 0x2d, 0xc8, 0xc2, 
//                                  0x0, 0x2e, 0xcc, 0xc1, 
//                                  0x0, 0x2f, 0xd0, 0xd2, 
//                                  0x0, 0x30, 0xd4, 0xd1, 
//                                  0x0, 0x31, 0xd8, 0xd2, 
//                                  0x0, 0x32, 0xdc, 0xd1, 
//                                  0x0, 0x33, 0xe0, 0xe2, 
//                                  0x0, 0x34, 0xe4, 0xe1,
//                                  0x0, 0x35, 0xe8, 0xe2, 
//                                  0x0, 0x36, 0xec, 0xe1, 
//                                  0x0, 0x37, 0xf0, 0xf2, 
//                                  0x0, 0x38, 0xf4, 0xf1, 
//                                  0x0, 0x39, 0xf8, 0xf2, 
//                                  0x0, 0x3a, 0xfc, 0xf1, 
//                                  };
//    uint16_t byte_array_hex_length = 254;
}

/**
 * @brief Function for demonstrating FATFS usage.
 */
void fatfs_init()
{
    static DIR dir;
    static FILINFO fno;
    static FATFS fs;
    
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    //NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        //NRF_LOG_INFO("Disk initialization failed.");
        printf("Disk initialization failed.\n");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;

    //NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        //NRF_LOG_INFO("Mount failed.");
        printf("Mount failed.\n");
        return;
    }

    //NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        //NRF_LOG_INFO("Directory listing failed!");
        printf("Directory listing failed!\n");
        return;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            //NRF_LOG_INFO("Directory read failed.");
            printf("Directory read failed.\n");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                //NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
                //NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, NRF_LOG_PUSH(fno.fname));
            }
        }
    }
    while (fno.fname[0]);
    //NRF_LOG_RAW_INFO("");
    printf("fatfs init complete\n");
    return;
}

void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;

    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

//    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
//        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
//    NRF_LOG_INFO("Capacity: %d MB", capacity);

//    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
    printf("mount failed");
//        NRF_LOG_INFO("Mount failed.");
        return;
    }

//    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
//        NRF_LOG_INFO("Directory listing failed!");
        return;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
//            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
//                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
//                NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
//    NRF_LOG_RAW_INFO("");

//    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
//        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
//        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
//        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    (void) f_close(&file);
    return;
}