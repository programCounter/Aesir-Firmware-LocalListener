/*
This file contains the source code to implement communication with a SD Card
The file system is tested with a fat32 format
The communication is on the SPI bus.

*/
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#define SDC_BUFFER_SIZE 512

typedef struct fatfs_write_buffer_t_ {
    uint16_t length;
    char data[SDC_BUFFER_SIZE];
} fatfs_write_buffer_t;

typedef struct BSI_Att_t_ {
      char BSI_Name[16];
      uint8_t Start_Time[8];
}BSI_Att_t;

typedef struct BSI_Data_t_ {
      uint16_t CountMin;
      uint16_t SensorValue;
      uint8_t SensorCh;
}BSI_Data_t;

extern BSI_Data_t BSI_Data; //should be defined in main of program when ported over to LOLI
extern BSI_Att_t BSI_Attribute;


void fatfs_example(void);
void fatfs_init(void);
void fatfs_write(fatfs_write_buffer_t fatfs_buffer);
void data_ch_decode(BSI_Data_t * tempValue);
void current_time(uint16_t minutes, char *CurrentTimeString, uint8_t *CurrentTime);
void fatfs_bsi_data_write(uint8_t *rx_data_8bit, uint16_t rx_length, bool first_rx);