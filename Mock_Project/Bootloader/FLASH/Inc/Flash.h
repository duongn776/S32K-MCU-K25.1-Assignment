#if!defined (__FLASH_H__)
#define __FLASH_H__


/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define CMD_PROGRAM_LONGWORD     (0x07)
#define CMD_ERASE_FLASH_SECTOR   (0x09)
#define APP_SECTOR_COUNT    	 (16u)
/**
 * @brief  Program alignment
 */
#define FTFC_WRITE_DOUBLE_WORD   (8U)
#define FTFC_P_FLASH_SECTOR_SIZE (0x1000)
#define WRITE_FUNCTION_ADDRESS    (0x1FFF8400)
void Mem_43_INFLS_IPW_LoadAc(void);

void Ftfc_AccessCode(void) __attribute__ ((section (".acmem_43_infls_code_rom")));
/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief
 * get address
 * @param Addr: input address
 * @return
 * return address
 */
uint32_t Read_FlashAddress(uint32_t Addr);

/*!
 * @brief
 * flash data input into flash
 * @param Addr: address to flash data to flash
 * @param *Data: input data need to flash data into flash
 * @return
 * return 1: if success
 */
uint8_t Program_LongWord_8B(uint32_t Addr,uint8_t *Data);

/*!
 * @brief
 * erase a sector in flash
 * @param Addr: address to erase
 * @return
 * return 1: if success
 */
uint8_t Erase_Sector(uint32_t Addr);

/*!
 * @brief
 * erase multi sectors in flash
 * @param Addr: address to erase
 * @return
 * return 1: if success
 */
uint8_t Erase_Multi_Sector(uint32_t Addr,uint8_t Size);

#endif
