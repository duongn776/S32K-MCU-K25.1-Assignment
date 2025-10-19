/*******************************************************************************
 * @file    Srec_Parser.c
 * @brief   Implementation of Motorola S-Record parser module.
 * @details This file contains helper functions to parse a single SREC line,
 *          extract address, data, and verify checksum integrity.
 *
 * @date    Oct 19, 2025
 * @author  nhduong
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "Srec_Parser.h"

/*******************************************************************************
 * Private Functions Prototypes
 ******************************************************************************/
static int32_t hexchar_to_value(char c);
static int32_t hexbyte_to_value(const char *hexbyte);

/*******************************************************************************
 * Private Function Definitions
 ******************************************************************************/

/**
 * @brief Convert a hexadecimal character to its integer value.
 *
 * @param c Hexadecimal character ('0'-'9', 'A'-'F', 'a'-'f').
 * @return int32_t The corresponding integer value, or -1 if invalid.
 */
static int32_t hexchar_to_value(char c)
{
    int32_t ret_val;

    /* Convert hex character to integer value */
    if ((c >= '0') && (c <= '9'))
    {
        ret_val = (int32_t)(c - '0');
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
        ret_val = (int32_t)(10 + (c - 'A'));
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
        ret_val = (int32_t)(10 + (c - 'a'));
    }
    else
    {
        /* Invalid hex character */
        ret_val = -1;
    }

    return ret_val;
}

/**
 * @brief Convert two hexadecimal characters to a byte value.
 *
 * @param hexbyte Pointer to a 2-character hexadecimal string.
 * @return int32_t The corresponding byte value (0–255), or -1 if invalid.
 */
static int32_t hexbyte_to_value(const char *hexbyte)
{
    int32_t high_nibble, low_nibble;

    /* Convert high nibble */
    high_nibble = hexchar_to_value(hexbyte[0]);

    /* Convert low nibble */
    low_nibble = hexchar_to_value(hexbyte[1]);

    /* Check for invalid hex digits */
    if ((high_nibble < 0) || (low_nibble < 0))
    {
        return -1;
    }

    /* Combine nibbles to form full byte */
    return ((high_nibble << 4) | low_nibble);
}

/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/
/**
 * @brief   Parse a single Motorola SREC record line.
 *
 * @param   line Pointer to null-terminated SREC line (e.g. "S1137AF00A0A0D00...").
 * @param   rec  Pointer to structure that will store parsed record data.
 *
 * @return  true if checksum and record are valid, false otherwise.
 *
 * @details
 * This function performs the following:
 *  1. Identifies record type (S0–S9)
 *  2. Extracts byte count, address, and data
 *  3. Computes checksum and compares it with provided one
 *  4. Fills SREC_Record structure with parsed information
 */
bool Srec_parse_line(const char *line, SREC_Record *rec)
{
    int32_t tmp = 0;
    uint8_t offset = 4U;
    uint8_t byte_count = 0U;
    uint8_t addr_len = 0U;
    int32_t data_len = 0;
    uint8_t checksum = 0U;
    uint8_t calc_checksum = 0U;
    uint32_t sum = 0U;

    /* Validate input */
    if ((line == NULL) || (line[0] != 'S'))
    {
        return false;
    }

    /***************************************************************************
     * Step 1: Identify record type (S0–S9)
     ***************************************************************************/
    rec->record_type = SREC_TYPE_UNKNOWN;

    switch (line[1])
    {
        case '0': rec->record_type = SREC_TYPE_S0; break;
        case '1': rec->record_type = SREC_TYPE_S1; break;
        case '2': rec->record_type = SREC_TYPE_S2; break;
        case '3': rec->record_type = SREC_TYPE_S3; break;
        case '5': rec->record_type = SREC_TYPE_S5; break;
        case '7': rec->record_type = SREC_TYPE_S7; break;
        case '8': rec->record_type = SREC_TYPE_S8; break;
        case '9': rec->record_type = SREC_TYPE_S9; break;
        default:  return false;
    }

    /***************************************************************************
     * Step 2: Parse byte count field
     ***************************************************************************/
    tmp = hexbyte_to_value(&line[2]);
    if (tmp < 0)
    {
        return false;
    }
    byte_count = (uint8_t)tmp;

    /***************************************************************************
     * Step 3: Determine address field length (depends on record type)
     ***************************************************************************/
    switch (rec->record_type)
    {
        case SREC_TYPE_S1:
        case SREC_TYPE_S5:
        case SREC_TYPE_S9: addr_len = 2U; break;

        case SREC_TYPE_S2:
        case SREC_TYPE_S8: addr_len = 3U; break;

        case SREC_TYPE_S3:
        case SREC_TYPE_S7: addr_len = 4U; break;

        default: addr_len = 2U; break;
    }

    /***************************************************************************
     * Step 4: Parse address field
     ***************************************************************************/
    rec->address = 0U;
    for (uint8_t i = 0U; i < addr_len; i++)
    {
        tmp = hexbyte_to_value(&line[offset]);
        if (tmp < 0)
        {
            return false;
        }

        rec->address = (rec->address << 8U) | (uint8_t)tmp;
        offset += 2U;
    }

    /***************************************************************************
     * Step 5: Parse data field
     ***************************************************************************/
    data_len = (int32_t)byte_count - (int32_t)addr_len - 1;
    if ((data_len < 0) || (data_len > SREC_MAX_DATA_LEN))
    {
        return false;
    }

    rec->data_length = (uint8_t)data_len;

    for (int i = 0; i < rec->data_length; i++)
    {
        tmp = hexbyte_to_value(&line[offset]);
        if (tmp < 0)
        {
            return false;
        }

        rec->data[i] = (uint8_t)tmp;
        offset += 2U;
    }

    /***************************************************************************
     * Step 6: Parse and verify checksum
     ***************************************************************************/
    tmp = hexbyte_to_value(&line[offset]);
    if (tmp < 0)
    {
        return false;
    }
    checksum = (uint8_t)tmp;

    /* Calculate checksum (sum of all bytes except checksum) */
    for (uint8_t i = 0U; i < (byte_count - 1U); i++)
    {
        tmp = hexbyte_to_value(&line[2 + (i * 2)]);
        if (tmp < 0)
        {
            return false;
        }

        sum += (uint32_t)tmp;
    }

    calc_checksum = (uint8_t)(0xFFU - (sum & 0xFFU));
    rec->checksum_ok = (calc_checksum == checksum);

    return rec->checksum_ok;
}

/*******************************************************************************
 * End of File
 ******************************************************************************/
