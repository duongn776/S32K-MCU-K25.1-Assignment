/**
 ******************************************************************************
 * @file    HAL_Common.h
 * @author  Nguyen Hoang Duong
 * @date    06-Oct-2025
 * @brief   Common definitions and base pointers for HAL GPIO/PORT drivers.
 ******************************************************************************
 */

#ifndef INC_HAL_COMMON_H_
#define INC_HAL_COMMON_H_

#include "S32K144.h"
#include <stddef.h>
#include <stdint.h>

/* ===========================================================
 *                      GENERAL DEFINITIONS
 * =========================================================== */

/**
 * @brief  Total number of PORT instances available on S32K144
 */
#define GPIO_MAX_PORTS          (5U)

/**
 * @brief  Total number of GPIO pins (5 ports × 32 pins)
 */
#define GPIO_MAX_PINS           (GPIO_MAX_PORTS * 32U)

/**
 * @brief  Check if pin index is within valid range
 */
#define PIN_IS_AVAILABLE(n)     ((n) < GPIO_MAX_PINS)

/**
 * @brief  Extract PORT index from pin number (0 = A, 1 = B, 2 = C, ...)
 */
#define GET_PORT(n)             ((n) / 32U)

/**
 * @brief  Extract pin number (0–31) within a given PORT
 */
#define GET_PIN(n)              ((n) % 32U)

/** @defgroup PORT_pins_define  PORT pins define
  * @{
  */

/* ===== PORT A (0–31) ===== */
#define PA0   (0)
#define PA1   (1)
#define PA2   (2)
#define PA3   (3)
#define PA4   (4)
#define PA5   (5)
#define PA6   (6)
#define PA7   (7)
#define PA8   (8)
#define PA9   (9)
#define PA10  (10)
#define PA11  (11)
#define PA12  (12)
#define PA13  (13)
#define PA14  (14)
#define PA15  (15)
#define PA16  (16)
#define PA17  (17)
#define PA18  (18)
#define PA19  (19)
#define PA20  (20)
#define PA21  (21)
#define PA22  (22)
#define PA23  (23)
#define PA24  (24)
#define PA25  (25)
#define PA26  (26)
#define PA27  (27)
#define PA28  (28)
#define PA29  (29)
#define PA30  (30)
#define PA31  (31)

/* ===== PORT B (32–63) ===== */
#define PB0   (32)
#define PB1   (33)
#define PB2   (34)
#define PB3   (35)
#define PB4   (36)
#define PB5   (37)
#define PB6   (38)
#define PB7   (39)
#define PB8   (40)
#define PB9   (41)
#define PB10  (42)
#define PB11  (43)
#define PB12  (44)
#define PB13  (45)
#define PB14  (46)
#define PB15  (47)
#define PB16  (48)
#define PB17  (49)
#define PB18  (50)
#define PB19  (51)
#define PB20  (52)
#define PB21  (53)
#define PB22  (54)
#define PB23  (55)
#define PB24  (56)
#define PB25  (57)
#define PB26  (58)
#define PB27  (59)
#define PB28  (60)
#define PB29  (61)
#define PB30  (62)
#define PB31  (63)

/* ===== PORT C (64–95) ===== */
#define PC0   (64)
#define PC1   (65)
#define PC2   (66)
#define PC3   (67)
#define PC4   (68)
#define PC5   (69)
#define PC6   (70)
#define PC7   (71)
#define PC8   (72)
#define PC9   (73)
#define PC10  (74)
#define PC11  (75)
#define PC12  (76)
#define PC13  (77)
#define PC14  (78)
#define PC15  (79)
#define PC16  (80)
#define PC17  (81)
#define PC18  (82)
#define PC19  (83)
#define PC20  (84)
#define PC21  (85)
#define PC22  (86)
#define PC23  (87)
#define PC24  (88)
#define PC25  (89)
#define PC26  (90)
#define PC27  (91)
#define PC28  (92)
#define PC29  (93)
#define PC30  (94)
#define PC31  (95)

/* ===== PORT D (96–127) ===== */
#define PD0   (96)
#define PD1   (97)
#define PD2   (98)
#define PD3   (99)
#define PD4   (100)
#define PD5   (101)
#define PD6   (102)
#define PD7   (103)
#define PD8   (104)
#define PD9   (105)
#define PD10  (106)
#define PD11  (107)
#define PD12  (108)
#define PD13  (109)
#define PD14  (110)
#define PD15  (111)
#define PD16  (112)
#define PD17  (113)
#define PD18  (114)
#define PD19  (115)
#define PD20  (116)
#define PD21  (117)
#define PD22  (118)
#define PD23  (119)
#define PD24  (120)
#define PD25  (121)
#define PD26  (122)
#define PD27  (123)
#define PD28  (124)
#define PD29  (125)
#define PD30  (126)
#define PD31  (127)

/* ===== PORT E (128–159) ===== */
#define PE0   (128)
#define PE1   (129)
#define PE2   (130)
#define PE3   (131)
#define PE4   (132)
#define PE5   (133)
#define PE6   (134)
#define PE7   (135)
#define PE8   (136)
#define PE9   (137)
#define PE10  (138)
#define PE11  (139)
#define PE12  (140)
#define PE13  (141)
#define PE14  (142)
#define PE15  (143)
#define PE16  (144)
#define PE17  (145)
#define PE18  (146)
#define PE19  (147)
#define PE20  (148)
#define PE21  (149)
#define PE22  (150)
#define PE23  (151)
#define PE24  (152)
#define PE25  (153)
#define PE26  (154)
#define PE27  (155)
#define PE28  (156)
#define PE29  (157)
#define PE30  (158)
#define PE31  (159)

/* ===========================================================
 *                      BASE POINTER ARRAYS
 * =========================================================== */

/**
 * @brief  External declarations for PORT and GPIO base address arrays.
 *         Defined in HAL_GPIO.c to avoid multiple-definition.
 */
extern PORT_Type * const PORT_BASE[];
extern GPIO_Type * const GPIO_BASE[];

#endif /* INC_HAL_COMMON_H_ */
