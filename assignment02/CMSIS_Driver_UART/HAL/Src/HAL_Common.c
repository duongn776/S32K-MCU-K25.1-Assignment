/*
 * HAL_Common.c
 *
 *  Created on: Oct 12, 2025
 *      Author: nhduong
 */

#include "HAL_Common.h"


int32_t myStrlen(const char *c)
{
    int32_t len = 0;
    while (*c != '\0')
    {
        len++;
        c++;
    }
    return len;
}

int myStrcmp(const char *s1, const char *s2)
{
    while (*s1 && (*s1 == *s2))
    {
        s1++;
        s2++;
    }

    return (unsigned char)(*s1) - (unsigned char)(*s2);
}

char myToUpper(char c)
{
    if (c >= 'a' && c <= 'z')
        c = c - ('a' - 'A');  // hoặc: c -= 32;
    return c;
}
