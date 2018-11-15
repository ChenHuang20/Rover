/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file can1.c
 *
 * @author hc <450801089@qq.com>
 *
 *
 ***************************************************************************/

#include "flash.h"
#include <string.h>
#include "topics.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart6;

static FLASH_EraseInitTypeDef EraseInitStruct;

static uint32_t SectorError = 404;

#define FLASH_CHECK 0xAA

typedef struct {
    uint32_t valid_check;
    float accel_offset[3];
    float gyro_offset[3];

} flash_params_t;

//uint32_t flash_read(uint32_t addr) {

//    return *(uint32_t *) addr;
//}

uint32_t GetSector(uint32_t Address) {

  uint32_t sector = 0;

    if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
    }
    else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
    }
    else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
    }
    else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
    }
    else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
    }
    else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
    }
    else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
    }
    else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
    }
    else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
    }
    else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
    }
    else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
    }
    else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11)) {
    sector = FLASH_SECTOR_11;
    }
    else if((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12)) {
    sector = FLASH_SECTOR_12;
    }
    else if((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13)) {
    sector = FLASH_SECTOR_13;
    }
    else if((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14)) {
    sector = FLASH_SECTOR_14;
    }
    else if((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15)) {
    sector = FLASH_SECTOR_15;
    }
    else if((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16)) {
    sector = FLASH_SECTOR_16;
    }
    else if((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17)) {
    sector = FLASH_SECTOR_17;
    }
    else if((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18)) {
    sector = FLASH_SECTOR_18;
    }
    else if((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19)) {
    sector = FLASH_SECTOR_19;
    }
    else if((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20)) {
    sector = FLASH_SECTOR_20;
    }
    else if((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21)) {
    sector = FLASH_SECTOR_21;
    }
    else if((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22)) {
    sector = FLASH_SECTOR_22;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23) */ {
    sector = FLASH_SECTOR_23;
    }
    return sector;
}

uint8_t flash_write(uint8_t* buff, uint32_t len) {
    //erase flash before program 
	uint32_t WriteSector = GetSector(PARAM_SAVED_START_ADDRESS);

	uint32_t start_addr = PARAM_SAVED_START_ADDRESS;

	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = WriteSector;
	EraseInitStruct.NbSectors = 1;

	if(HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) )
		while(1);

	//start program flash,write data in here
	while (len -- ) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_addr, *buff);
		start_addr++;
		buff++;
	}

	HAL_FLASH_Lock(); 
	return 0;
}

void erase_sector_test(uint8_t sector_number) {

    uint32_t SectorError = 0;

    FLASH_EraseInitTypeDef flash_erase_init;

    HAL_FLASH_Unlock();

    flash_erase_init.Sector = sector_number;
    flash_erase_init.NbSectors = 1;
    flash_erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_OK != HAL_FLASHEx_Erase(&flash_erase_init, &SectorError) )
		while(1);

    HAL_FLASH_Lock();
}

uint8_t BSP_FLASH_Read(uint8_t* buff, uint32_t len) {

    HAL_UART_Transmit(&huart6,buff,len,1000);

	return 0;
}

void flash_save()
{
    flash_params_t flash_params = { .valid_check = FLASH_CHECK };

    for (int i = 0; i < 3; i++) {
        flash_params.accel_offset[i] = _params.accel_offset[i];
        flash_params.gyro_offset[i] = _params.gyro_offset[i];
    }

    flash_write((uint8_t*)&flash_params, sizeof(flash_params_t));
}

void flash_read()
{
    flash_params_t flash_params = { 0 };

    memcpy((void*)&flash_params, (void*)PARAM_SAVED_START_ADDRESS, sizeof(flash_params_t));

    // simple valid check
    if (flash_params.valid_check == FLASH_CHECK) {
        for (int i = 0; i < 3; i++) {
            _params.accel_offset[i] = flash_params.accel_offset[i];
            _params.gyro_offset[i] = flash_params.gyro_offset[i];
        }
    }
}
