/*
 * Flash.c
 *
 *  Created on: Nov 20, 2025
 *      Author: nhduong
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "FLASH.h"

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Get address*/
uint32_t Read_FlashAddress(uint32_t Addr)
{
    return *(__IO uint32_t*)Addr;
}

/* Program Address and Data (8bit pointer) into Flash Memory */
uint8_t Program_LongWord_8B(uint32_t Addr,uint8_t *Data)
{
    /* wait previous cmd finish */
    while (IP_FTFC->FSTAT == 0x00);

    /* clear previous cmd error */
    if(IP_FTFC->FSTAT != 0x80)
    {
        IP_FTFC->FSTAT = 0x30;
    }
    /* Program 4 bytes in a program flash block */
    IP_FTFC->FCCOB[3] = CMD_PROGRAM_LONGWORD;

    /* fill Address */
    IP_FTFC->FCCOB[2] = (uint8_t)(Addr >> 16);
    IP_FTFC->FCCOB[1] = (uint8_t)(Addr >> 8);
    IP_FTFC->FCCOB[0] = (uint8_t)(Addr >> 0);

    /* fill Data */
    IP_FTFC->FCCOB[7] = (uint8_t)(Data[3]);
    IP_FTFC->FCCOB[6] = (uint8_t)(Data[2]);
    IP_FTFC->FCCOB[5] = (uint8_t)(Data[1]);
    IP_FTFC->FCCOB[4] = (uint8_t)(Data[0]);

    /* load the second word */
    IP_FTFC->FCCOB[11] = (uint8_t)(Data[7]);
    IP_FTFC->FCCOB[10] = (uint8_t)(Data[6]);
    IP_FTFC->FCCOB[9]  = (uint8_t)(Data[5]);
    IP_FTFC->FCCOB[8]  = (uint8_t)(Data[4]);

    /* wait until operation finishes or write/erase timeout is reached */
    while (IP_FTFC->FSTAT == 0x00);
    /* Clear CCIF */
    IP_FTFC->FSTAT = 0x80;
    asm("nop");
    asm("nop");
    /* wait until operation finishes or write/erase timeout is reached */
//    while (0U == ((IP_FTFC->FSTAT) & 0x80));
    return 1;
}

/* Erase a flash Sector */
uint8_t  Erase_Sector(uint32_t Addr)
{
    /* wait previous cmd finish */
    while (IP_FTFC->FSTAT == 0x00);

    /* clear previous cmd error */
    if(IP_FTFC->FSTAT != 0x80)
    {
        IP_FTFC->FSTAT = 0x30;
    }
    /* Erase all bytes in a program flash sector */
    IP_FTFC->FCCOB[3] = CMD_ERASE_FLASH_SECTOR;

    /* fill Address */
    IP_FTFC->FCCOB[2] = (uint8_t)(Addr >> 16);
    IP_FTFC->FCCOB[1] = (uint8_t)(Addr >> 8);
    IP_FTFC->FCCOB[0] = (uint8_t)(Addr >> 0);

    /* wait until operation finishes or write/erase timeout is reached */
    while (IP_FTFC->FSTAT == 0x00);
    /* Clear CCIF */
    IP_FTFC->FSTAT = 0x80;
    asm("nop");
    asm("nop");
    return 1;
}

/* Erase all flash sector */
uint8_t  Erase_Multi_Sector(uint32_t Addr,uint8_t Size)
{
    uint8_t i;
    for(i = 0; i < Size; i++)
    {
        Erase_Sector(Addr + i*FTFC_P_FLASH_SECTOR_SIZE);
    }
    return 1;
}
void FTFC_IRQHandler(void)
{
	/* */
}



