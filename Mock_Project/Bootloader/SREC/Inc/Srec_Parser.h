/*******************************************************************************
 * @file    Srec_Parser.h
 * @brief   Header file for Motorola S-Record parser module.
 * @details This module provides data structures and function declarations
 *          for parsing SREC formatted records used in Bootloader applications.
 *
 * @date    Oct 19, 2025
 * @author  nhduong
 ******************************************************************************/

#ifndef INC_SREC_PARSER_H_
#define INC_SREC_PARSER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "S32K144.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SREC_MAX_DATA_LEN    255U  /*!< Maximum data length for a single SREC record */
typedef uint8_t boolean;
#define NULL ((void *)0)
#define true 1
#define false 0
/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @enum SREC_RecordType_t
 * @brief SREC record type identifiers as defined by Motorola S-Record format.
 */
typedef enum
{
    SREC_TYPE_S0 = 0,      /*!< Header record (typically contains information) */
    SREC_TYPE_S1 = 1,      /*!< Data record with 2-byte (16-bit) address */
    SREC_TYPE_S2 = 2,      /*!< Data record with 3-byte (24-bit) address */
    SREC_TYPE_S3 = 3,      /*!< Data record with 4-byte (32-bit) address */
    SREC_TYPE_S5 = 5,      /*!< Count record (number of S1/S2/S3 records) */
    SREC_TYPE_S7 = 7,      /*!< Termination record for S3 (4-byte start address) */
    SREC_TYPE_S8 = 8,      /*!< Termination record for S2 (3-byte start address) */
    SREC_TYPE_S9 = 9,      /*!< Termination record for S1 (2-byte start address) */
    SREC_TYPE_UNKNOWN = 0xFF /*!< Unknown or invalid record type */
} SREC_RecordType_t;

/**
 * @struct SREC_Record
 * @brief Structure to hold parsed information from a single SREC line.
 */
typedef struct
{
    SREC_RecordType_t record_type;              /*!< Type of the SREC record */
    uint32_t address;                           /*!< Address extracted from record */
    uint8_t  data[SREC_MAX_DATA_LEN];           /*!< Data bytes from record */
    uint8_t  data_length;                       /*!< Number of valid data bytes */
    boolean     checksum_ok;                       /*!< Checksum verification result */
} SREC_Record;

/*******************************************************************************
 * API Prototypes
 ******************************************************************************/

/**
 * @brief   Parse a single SREC line and extract record information.
 * @param   line Pointer to a null-terminated SREC line string (e.g. "S1137AF0...").
 * @param   rec  Pointer to a SREC_Record structure to store parsed results.
 * @return  true if the record is valid and checksum passes, false otherwise.
 */
boolean Srec_parse_line(const char *line, SREC_Record *rec);

#endif /* INC_SREC_PARSER_H_ */

/*******************************************************************************
 * End of File
 ******************************************************************************/
