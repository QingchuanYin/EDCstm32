#include "oled.h"

#include <string.h>

static const uint8_t OLED_Font6x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x10,0x08,0x08,0x10,0x08}
};

static uint8_t OLED_Framebuffer[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t OLED_DirtyMin[OLED_PAGE_COUNT];
static uint8_t OLED_DirtyMax[OLED_PAGE_COUNT];

__STATIC_FORCEINLINE void OLED_Delay(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

__STATIC_FORCEINLINE void OLED_SCL(uint8_t level)
{
    if (level != 0u)
    {
        OLED_SCL_GPIO_PORT->BSRR = OLED_SCL_GPIO_PIN;
    }
    else
    {
        OLED_SCL_GPIO_PORT->BRR = OLED_SCL_GPIO_PIN;
    }
}

__STATIC_FORCEINLINE void OLED_SDA(uint8_t level)
{
    if (level != 0u)
    {
        OLED_SDA_GPIO_PORT->BSRR = OLED_SDA_GPIO_PIN;
    }
    else
    {
        OLED_SDA_GPIO_PORT->BRR = OLED_SDA_GPIO_PIN;
    }
}

static void OLED_I2C_Start(void)
{
    OLED_SDA(1);
    OLED_SCL(1);
    OLED_Delay();
    OLED_SDA(0);
    OLED_Delay();
    OLED_SCL(0);
}

static void OLED_I2C_Stop(void)
{
    OLED_SDA(0);
    OLED_SCL(1);
    OLED_Delay();
    OLED_SDA(1);
    OLED_Delay();
}

static void OLED_I2C_WaitAck(void)
{
    OLED_SDA(1);
    OLED_Delay();
    OLED_SCL(1);
    OLED_Delay();
    OLED_SCL(0);
}

static void OLED_I2C_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        OLED_SDA((byte & 0x80u) != 0u);
        OLED_Delay();
        OLED_SCL(1);
        OLED_Delay();
        OLED_SCL(0);
        byte <<= 1;
    }
    OLED_I2C_WaitAck();
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDR);
    OLED_I2C_SendByte(0x00);
    OLED_I2C_SendByte(command);
    OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDR);
    OLED_I2C_SendByte(0x40);
    OLED_I2C_SendByte(data);
    OLED_I2C_Stop();
}

static void OLED_WriteDataBlock(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_I2C_WRITE_ADDR);
    OLED_I2C_SendByte(0x40);
    for (i = 0u; i < length; i++)
    {
        OLED_I2C_SendByte(data[i]);
    }
    OLED_I2C_Stop();
}

static void OLED_ResetDirty(uint8_t page)
{
    OLED_DirtyMin[page] = OLED_WIDTH;
    OLED_DirtyMax[page] = 0u;
}

static void OLED_MarkDirty(uint8_t page, uint8_t x)
{
    if (x < OLED_DirtyMin[page])
    {
        OLED_DirtyMin[page] = x;
    }
    if (x > OLED_DirtyMax[page])
    {
        OLED_DirtyMax[page] = x;
    }
}

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    OLED_SCL(1);
    OLED_SDA(1);

    GPIO_InitStruct.Pin = OLED_SCL_GPIO_PIN | OLED_SDA_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_SCL_GPIO_PORT, &GPIO_InitStruct);
}

void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_GPIO_Init();

    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x02);
    OLED_WriteCommand(0xB0);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);
    /* Segment remap: 0xA0=normal, 0xA1=flipped */
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);

    OLED_Clear();
}

void OLED_SetPos(uint8_t x, uint8_t page)
{
    if (x >= OLED_WIDTH)
    {
        x = OLED_WIDTH - 1u;
    }
    if (page >= OLED_PAGE_COUNT)
    {
        page = OLED_PAGE_COUNT - 1u;
    }

    OLED_WriteCommand((uint8_t)(0xB0u + page));
    OLED_WriteCommand((uint8_t)(0x00u | (x & 0x0Fu)));
    OLED_WriteCommand((uint8_t)(0x10u | ((x >> 4) & 0x0Fu)));
}

void OLED_Fill(uint8_t data)
{
    uint8_t page;
    uint8_t x;

    for (page = 0; page < OLED_PAGE_COUNT; page++)
    {
        OLED_SetPos(0, page);
        for (x = 0; x < OLED_WIDTH; x++)
        {
            OLED_Framebuffer[page][x] = data;
        }
        OLED_WriteDataBlock(OLED_Framebuffer[page], OLED_WIDTH);
        OLED_ResetDirty(page);
    }
}

void OLED_Clear(void)
{
    OLED_Fill(0x00);
}

void OLED_ShowChar(uint8_t x, uint8_t page, char ch)
{
    uint8_t i;
    uint8_t index;

    if ((x > (OLED_WIDTH - 6u)) || (page >= OLED_PAGE_COUNT))
    {
        return;
    }

    if ((ch < ' ') || (ch > '~'))
    {
        ch = '?';
    }

    index = (uint8_t)(ch - ' ');
    OLED_SetPos(x, page);
    for (i = 0; i < 5; i++)
    {
        OLED_WriteData(OLED_Font6x8[index][i]);
    }
    OLED_WriteData(0x00);
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
    while ((*str != '\0') && (page < OLED_PAGE_COUNT))
    {
        if (x > (OLED_WIDTH - 6u))
        {
            x = 0;
            page++;
            continue;
        }

        OLED_ShowChar(x, page, *str);
        x = (uint8_t)(x + 6u);
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len)
{
    uint8_t i;
    uint32_t div = 1u;

    if (len == 0u)
    {
        return;
    }

    for (i = 1; i < len; i++)
    {
        div *= 10u;
    }

    for (i = 0; i < len; i++)
    {
        OLED_ShowChar((uint8_t)(x + i * 6u), page, (char)('0' + (num / div) % 10u));
        div /= 10u;
    }
}

void OLED_ShowSignedNum(uint8_t x, uint8_t page, int32_t num, uint8_t len)
{
    uint32_t abs_num;

    if (num < 0)
    {
        OLED_ShowChar(x, page, '-');
        abs_num = (uint32_t)(-num);
    }
    else
    {
        OLED_ShowChar(x, page, '+');
        abs_num = (uint32_t)num;
    }

    OLED_ShowNum((uint8_t)(x + 6u), page, abs_num, len);
}

void OLED_BufferClear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0u; page < OLED_PAGE_COUNT; page++)
    {
        for (x = 0u; x < OLED_WIDTH; x++)
        {
            if (OLED_Framebuffer[page][x] != 0u)
            {
                OLED_Framebuffer[page][x] = 0u;
                OLED_MarkDirty(page, x);
            }
        }
    }
}

static void OLED_BufferSetPixel(uint8_t x, uint8_t y, uint8_t enabled)
{
    uint8_t mask;
    uint8_t page;
    uint8_t old_value;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    page = y >> 3;
    mask = (uint8_t)(1u << (y & 0x07u));
    old_value = OLED_Framebuffer[page][x];
    if (enabled != 0u)
    {
        OLED_Framebuffer[page][x] |= mask;
    }
    else
    {
        OLED_Framebuffer[page][x] &= (uint8_t)~mask;
    }
    if (OLED_Framebuffer[page][x] != old_value)
    {
        OLED_MarkDirty(page, x);
    }
}

void OLED_BufferShowChar(uint8_t x, uint8_t y, char ch)
{
    uint8_t column;
    uint8_t row;
    uint8_t index;

    if ((x > (OLED_WIDTH - 6u)) || (y > (OLED_HEIGHT - 7u)))
    {
        return;
    }
    if ((ch < ' ') || (ch > '~'))
    {
        ch = '?';
    }
    index = (uint8_t)(ch - ' ');

    for (column = 0u; column < 6u; column++)
    {
        uint8_t glyph_column = (column < 5u) ? OLED_Font6x8[index][column] : 0u;

        for (row = 0u; row < 7u; row++)
        {
            OLED_BufferSetPixel((uint8_t)(x + column), (uint8_t)(y + row),
                                (uint8_t)(glyph_column & (uint8_t)(1u << row)));
        }
    }
}

void OLED_BufferShowString(uint8_t x, uint8_t y, const char *str)
{
    if (str == NULL)
    {
        return;
    }

    while ((*str != '\0') && (x <= (OLED_WIDTH - 6u)))
    {
        OLED_BufferShowChar(x, y, *str);
        x = (uint8_t)(x + 6u);
        str++;
    }
}

void OLED_BufferFlush(void)
{
    uint8_t page;

    for (page = 0u; page < OLED_PAGE_COUNT; page++)
    {
        if (OLED_DirtyMin[page] < OLED_WIDTH)
        {
            uint8_t first = OLED_DirtyMin[page];
            uint16_t length = (uint16_t)OLED_DirtyMax[page] - first + 1u;

            OLED_SetPos(first, page);
            OLED_WriteDataBlock(&OLED_Framebuffer[page][first], length);
            OLED_ResetDirty(page);
        }
    }
}

void OLED_ShowFullScreenOne(void)
{
    uint8_t page;
    uint8_t x;
    uint8_t bit;

    for (page = 0; page < OLED_PAGE_COUNT; page++)
    {
        OLED_SetPos(0, page);
        for (x = 0; x < OLED_WIDTH; x++)
        {
            uint8_t data = 0u;

            for (bit = 0; bit < 8u; bit++)
            {
                uint8_t y = (uint8_t)(page * 8u + bit);
                uint8_t diagonal_left = (uint8_t)(62u - y);
                uint8_t pixel_on = 0u;

                if ((x >= 56u) && (x <= 79u) && (y >= 6u) && (y <= 55u))
                {
                    pixel_on = 1u;
                }
                else if ((y >= 14u) && (y <= 30u) &&
                         (x >= diagonal_left) && (x <= 63u))
                {
                    pixel_on = 1u;
                }
                else if ((y >= 56u) && (x >= 28u) && (x <= 107u))
                {
                    pixel_on = 1u;
                }

                if (pixel_on != 0u)
                {
                    data |= (uint8_t)(1u << bit);
                }
            }

            OLED_WriteData(data);
        }
    }
}

void OLED_Menu_Show(uint8_t cursor)
{
    uint8_t i;
    const char *menu_items[] = {
        "Motor Test",
        "K230 Track",
        "Mode 3"
    };

    if (cursor > 2u)
    {
        cursor = 0u;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "OLED MENU");
    OLED_ShowString(0, 1, "K2:Move K1:OK");

    for (i = 0; i < 3u; i++)
    {
        OLED_ShowChar(0, (uint8_t)(3u + i), (i == cursor) ? '>' : ' ');
        OLED_ShowString(12, (uint8_t)(3u + i), menu_items[i]);
    }
}

void OLED_Menu_ShowMode(uint8_t mode)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "Current Mode");
    OLED_ShowString(0, 2, "Mode:");
    OLED_ShowNum(36, 2, mode, 1);
    if (mode == 1u)
    {
        OLED_ShowString(0, 4, "K1:Motor1");
        OLED_ShowString(0, 5, "K2:Motor2");
    }
    else if (mode == 2u)
    {
        OLED_ShowString(0, 4, "Search target");
        OLED_ShowString(0, 5, "PA5 Relay");
        OLED_ShowString(0, 6, "K2:Exit");
    }
    else
    {
        OLED_ShowString(0, 5, "K2:Exit");
    }
}
