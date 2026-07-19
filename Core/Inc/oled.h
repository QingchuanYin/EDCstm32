#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define OLED_WIDTH                 128u
#define OLED_HEIGHT                64u
#define OLED_PAGE_COUNT            8u

/* 0x3C is the common 7-bit SSD1306 address; I2C write byte is 0x78. */
#define OLED_I2C_WRITE_ADDR        0x78u

#define OLED_SCL_GPIO_PORT         OLED_SCL_GPIO_Port
#define OLED_SCL_GPIO_PIN          OLED_SCL_Pin
#define OLED_SDA_GPIO_PORT         OLED_SDA_GPIO_Port
#define OLED_SDA_GPIO_PIN          OLED_SDA_Pin

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t data);
void OLED_SetPos(uint8_t x, uint8_t page);
void OLED_ShowChar(uint8_t x, uint8_t page, char ch);
void OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len);
void OLED_ShowSignedNum(uint8_t x, uint8_t page, int32_t num, uint8_t len);
void OLED_ShowFullScreenOne(void);
void OLED_Menu_Show(uint8_t cursor);
void OLED_Menu_ShowMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
