/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "S32K144.h"
#include "FLASH.h"
extern const uint32_t Mem_43_INFLS_ACWriteRomStart;
extern const uint32_t Mem_43_INFLS_ACWriteSize;
typedef void (*Mem_43_INFLS_AcWritePtrType)  (void);
#define MEM_43_INFLS_ARM_FAR_CALL2THUMB_CODE_BIT0_U32 (0x00000001UL)
/* Macro for Access Code Call. On ARM/Thumb, BLX instruction used by the compiler for calling a function
pointed to by the pointer requires that LSB bit of the address is set to one if the called fcn is coded in Thumb. */
#define MEM_43_INFLS_AC_CALL(ptr2fcn, ptr2fcnType) ((ptr2fcnType)(((uint32_t)(ptr2fcn)) | MEM_43_INFLS_ARM_FAR_CALL2THUMB_CODE_BIT0_U32))
/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Get address*/
uint32_t Read_FlashAddress(uint32_t Addr)
{
    return *(__IO uint32_t*)Addr;
}

void Ftfc_AccessCode(void)
{
    /* Clear CCIF */
    IP_FTFC->FSTAT = 0x80;
    /* wait until operation finishes or write/erase timeout is reached */
    while (IP_FTFC->FSTAT == 0x00);
}

void Mem_43_INFLS_IPW_LoadAc(void)
{
    uint32_t        Count;
    uint32_t        AcSize;  /* Word size */
    uint32_t       *RamPtr;
    const uint32_t *RomPtr;

    /* MEM_43_INFLS_JOB_WRITE */
    RomPtr = (const uint32_t *)((uint32_t)(&Mem_43_INFLS_ACWriteRomStart));
    RamPtr = (uint32_t*)((uint32_t)(WRITE_FUNCTION_ADDRESS));
    AcSize = (uint32_t)(&Mem_43_INFLS_ACWriteSize);

    /* Copy erase or write access code to RAM */
    /* AcSize is dynamically calculated and might not be multiple of 4U */
    for (Count = 0U; Count < AcSize; Count++)
    {
        /* Copy 4 bytes at a time*/
        RamPtr[Count] = RomPtr[Count];
    }
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
    MEM_43_INFLS_AC_CALL(WRITE_FUNCTION_ADDRESS, Mem_43_INFLS_AcWritePtrType)();

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
    MEM_43_INFLS_AC_CALL(WRITE_FUNCTION_ADDRESS, Mem_43_INFLS_AcWritePtrType)();
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
