#include "lcd.h"

/* =========================================================================
 * 핀 배치 (Nucleo-F429ZI)
 *   PE12 -> SPI4_SCK  (LCD의 SCL)
 *   PE14 -> SPI4_MOSI (LCD의 SDA)
 *   PD14 -> CS
 *   PD15 -> DC
 *   PG2  -> RST
 *
 * 이 파일에 있는 건 전부 이 모듈의 "구현 세부사항"입니다. main.c는
 * lcd_dashboard.h에 선언된 함수 6개만 알면 되고, 핀 번호/레지스터/폰트
 * 비트맵 같은 건 몰라도 됩니다 (전부 static = 이 파일 밖에서 안 보임).
 * ========================================================================= */

#define LCD_SPI                 SPI4
#define LCD_SPI_GPIO            GPIOE
#define LCD_SPI_AF              5

#define LCD_SCK_PIN             12
#define LCD_MOSI_PIN            14

#define LCD_CS_PORT             GPIOD
#define LCD_CS_PIN              14

#define LCD_DC_PORT             GPIOD
#define LCD_DC_PIN              15

#define LCD_RST_PORT            GPIOG
#define LCD_RST_PIN             2

/* 패널 네이티브 해상도 (리셋 시퀀스에서만 사용, 실제 좌표계는 LCD_W/LCD_H) */
#define LCD_WIDTH             128
#define LCD_HEIGHT            160

#define LCD_BLACK            0x0000
#define LCD_WHITE            0xFFFF
#define LCD_RED              0xF800
#define LCD_GREEN            0x07E0
#define LCD_BLUE             0x001F
#define LCD_YELLOW           0xFFE0
#define LCD_CYAN             0x07FF

/* ============================ ST7735 명령어 ============================ */
#define LCD_SWRESET  0x01
#define LCD_SLPOUT   0x11
#define LCD_INVOFF   0x20
#define LCD_DISPON   0x29
#define LCD_CASET    0x2A
#define LCD_RASET    0x2B
#define LCD_RAMWR    0x2C
#define LCD_MADCTL   0x36
#define LCD_COLMOD   0x3A
#define LCD_FRMCTR1  0xB1
#define LCD_FRMCTR2  0xB2
#define LCD_FRMCTR3  0xB3
#define LCD_INVCTR   0xB4
#define LCD_PWCTR1   0xC0
#define LCD_PWCTR2   0xC1
#define LCD_PWCTR3   0xC2
#define LCD_PWCTR4   0xC3
#define LCD_PWCTR5   0xC4
#define LCD_VMCTR1   0xC5
#define LCD_GMCTRP1  0xE0
#define LCD_GMCTRN1  0xE1

#define CS_LOW()   (LCD_CS_PORT->BSRR  = (1U << (LCD_CS_PIN + 16)))
#define CS_HIGH()  (LCD_CS_PORT->BSRR  = (1U << LCD_CS_PIN))
#define DC_LOW()   (LCD_DC_PORT->BSRR  = (1U << (LCD_DC_PIN + 16)))
#define DC_HIGH()  (LCD_DC_PORT->BSRR  = (1U << LCD_DC_PIN))
#define RST_LOW()  (LCD_RST_PORT->BSRR = (1U << (LCD_RST_PIN + 16)))
#define RST_HIGH() (LCD_RST_PORT->BSRR = (1U << LCD_RST_PIN))

#define COLOR_BG   LCD_BLACK

/* ---------------------------- 1) GPIO / SPI 저수준 ---------------------------- *
 * 여기 3개 함수는 "핀을 어떻게 설정하고 바이트를 어떻게 SPI로 내보내는가"만
 * 담당합니다. 위에서 그림을 그리는 코드는 이 계층을 전혀 신경 쓰지 않습니다. */

static void GPIO_SetAF(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    port->MODER   &= ~(0x3U << (pin * 2));
    port->MODER   |=  (0x2U << (pin * 2));
    port->OSPEEDR |=  (0x3U << (pin * 2));
    port->OTYPER  &= ~(0x1U << pin);
    port->PUPDR   &= ~(0x3U << (pin * 2));

    if (pin < 8) {
        port->AFR[0] &= ~(0xFU << (pin * 4));
        port->AFR[0] |=  ((uint32_t)af << (pin * 4));
    } else {
        port->AFR[1] &= ~(0xFU << ((pin - 8) * 4));
        port->AFR[1] |=  ((uint32_t)af << ((pin - 8) * 4));
    }
}

static void GPIO_SetOutput(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER   &= ~(0x3U << (pin * 2));
    port->MODER   |=  (0x1U << (pin * 2));
    port->OSPEEDR |=  (0x3U << (pin * 2));
    port->OTYPER  &= ~(0x1U << pin);
    port->PUPDR   &= ~(0x3U << (pin * 2));
}

static void SPI_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOGEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

    GPIO_SetAF(LCD_SPI_GPIO, LCD_SCK_PIN, LCD_SPI_AF);
    GPIO_SetAF(LCD_SPI_GPIO, LCD_MOSI_PIN, LCD_SPI_AF);

    GPIO_SetOutput(LCD_CS_PORT, LCD_CS_PIN);
    GPIO_SetOutput(LCD_DC_PORT, LCD_DC_PIN);
    GPIO_SetOutput(LCD_RST_PORT, LCD_RST_PIN);

    CS_HIGH();
    DC_HIGH();
    RST_HIGH();

    LCD_SPI->CR1 = 0;
    LCD_SPI->CR1 |= SPI_CR1_MSTR;
    LCD_SPI->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    /* [진단용] fPCLK/256, 최대한 느리게. 화면/폰트 렌더링 정상 확인 후에는
     * BR 비트를 낮춰서(예: 0x2~0x3) SPI 클럭을 올리는 것을 권장합니다. */
    LCD_SPI->CR1 |= (0x2U << SPI_CR1_BR_Pos);
    LCD_SPI->CR1 |= SPI_CR1_SPE;
}

static inline void SPI_SendByte(uint8_t data)
{
    while (!(LCD_SPI->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&LCD_SPI->DR = data;
    while (!(LCD_SPI->SR & SPI_SR_TXE)) { }
    while (LCD_SPI->SR & SPI_SR_BSY) { }
}

/* ------------------------- 2) ST7735 명령/데이터 계층 ------------------------- *
 * SPI 바이트를 "이건 명령이다(DC low)" / "이건 데이터다(DC high)"로 감싸는 계층. */

static void LCD_WriteCommand(uint8_t cmd)
{
    CS_LOW();
    DC_LOW();
    SPI_SendByte(cmd);
    CS_HIGH();
}

static void LCD_WriteData(uint8_t data)
{
    CS_LOW();
    DC_HIGH();
    SPI_SendByte(data);
    CS_HIGH();
}

void LCD_Init(void)
{
    SPI_Init();

    RST_HIGH();
    HAL_Delay(10);
    RST_LOW();
    HAL_Delay(50);
    RST_HIGH();
    HAL_Delay(150);

    LCD_WriteCommand(LCD_SWRESET);
    HAL_Delay(150);

    LCD_WriteCommand(LCD_SLPOUT);
    HAL_Delay(255);

    LCD_WriteCommand(LCD_FRMCTR1);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);

    LCD_WriteCommand(LCD_FRMCTR2);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);

    LCD_WriteCommand(LCD_FRMCTR3);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);

    LCD_WriteCommand(LCD_INVCTR);
    LCD_WriteData(0x07);

    LCD_WriteCommand(LCD_PWCTR1);
    LCD_WriteData(0xA2); LCD_WriteData(0x02); LCD_WriteData(0x84);

    LCD_WriteCommand(LCD_PWCTR2);
    LCD_WriteData(0xC5);

    LCD_WriteCommand(LCD_PWCTR3);
    LCD_WriteData(0x0A); LCD_WriteData(0x00);

    LCD_WriteCommand(LCD_PWCTR4);
    LCD_WriteData(0x8A); LCD_WriteData(0x2A);

    LCD_WriteCommand(LCD_PWCTR5);
    LCD_WriteData(0x8A); LCD_WriteData(0xEE);

    LCD_WriteCommand(LCD_VMCTR1);
    LCD_WriteData(0x0E);

    LCD_WriteCommand(LCD_INVOFF);

    /* ---- 랜드스케이프(160x128) 회전 ----
     * MV(0x20) | MX(0x40) = 0x60 (RGB 순서).
     * RED가 파랑, YELLOW가 하늘색(시안)으로 나오면 R/B가 뒤바뀐 것이라
     * BGR 비트(0x08)를 빼야 합니다 (지금 이 값 0x60이 그 상태).
     * 색은 맞는데 화면이 좌우/상하로 뒤집히면 0x60 대신 0xA0/0x00/0xC0 등으로
     * MY/MX 조합만 바꿔가며 맞추면 됩니다. */
    LCD_WriteCommand(LCD_MADCTL);
    LCD_WriteData(0x60);

    LCD_WriteCommand(LCD_COLMOD);
    LCD_WriteData(0x05);

    /* 논리 해상도 기준(가로 160 x 세로 128)으로 어드레스 윈도우 설정 */
    LCD_WriteCommand(LCD_CASET);
    LCD_WriteData(0x00); LCD_WriteData(0x00);
    LCD_WriteData(0x00); LCD_WriteData(LCD_W - 1);

    LCD_WriteCommand(LCD_RASET);
    LCD_WriteData(0x00); LCD_WriteData(0x00);
    LCD_WriteData(0x00); LCD_WriteData(LCD_H - 1);

    LCD_WriteCommand(LCD_GMCTRP1);
    {
        static const uint8_t gp[] = {0x02,0x1c,0x07,0x12,0x37,0x32,0x29,0x2d,
                                      0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
        for (uint8_t i = 0; i < sizeof(gp); i++) LCD_WriteData(gp[i]);
    }

    LCD_WriteCommand(LCD_GMCTRN1);
    {
        static const uint8_t gn[] = {0x03,0x1d,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                                      0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
        for (uint8_t i = 0; i < sizeof(gn); i++) LCD_WriteData(gn[i]);
    }

    LCD_WriteCommand(LCD_DISPON);
    HAL_Delay(100);
}

static void LCD_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    LCD_WriteCommand(LCD_CASET);
    LCD_WriteData(0x00); LCD_WriteData((uint8_t)x0);
    LCD_WriteData(0x00); LCD_WriteData((uint8_t)x1);

    LCD_WriteCommand(LCD_RASET);
    LCD_WriteData(0x00); LCD_WriteData((uint8_t)y0);
    LCD_WriteData(0x00); LCD_WriteData((uint8_t)y1);

    LCD_WriteCommand(LCD_RAMWR);
}

/* ---------------------- 3) 도형 프리미티브 (전부 FillRect 위에 구현) ---------------------- *
 * ST7735_FillRect가 유일하게 실제로 SPI에 픽셀 데이터를 쏘는 함수이고,
 * DrawPixel/DrawLine/FillCircle/DrawCircle/FillTriangle은 전부 이 함수를
 * 반복 호출하는 소프트웨어 래스터라이저입니다. 하드웨어 계층을 새로 만들
 * 필요 없이 도형만 늘려갈 수 있는 구조입니다. */

static void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((x >= LCD_W) || (y >= LCD_H)) return;
    if ((x + w - 1) >= LCD_W) w = LCD_W - x;
    if ((y + h - 1) >= LCD_H) h = LCD_H - y;

    LCD_SetAddrWindow(x, y, x + w - 1, y + h - 1);

    CS_LOW();
    DC_HIGH();
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        SPI_SendByte(hi);
        SPI_SendByte(lo);
    }
    CS_HIGH();
}

static void LCD_FillScreen(uint16_t color)
{
    LCD_FillRect(0, 0, LCD_W, LCD_H, color);
}

static inline void LCD_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= LCD_W || y >= LCD_H) return;
    LCD_FillRect((uint16_t)x, (uint16_t)y, 1, 1, color);
}

static inline int16_t abs16(int16_t v) { return v < 0 ? (int16_t)(-v) : v; }

/* 브레젠험 직선 (보행자 아이콘의 팔다리처럼 각진 선을 그릴 때 사용) */
static void LCD_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx = abs16((int16_t)(x1 - x0));
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (int16_t)(-abs16((int16_t)(y1 - y0)));
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    for (;;) {
        LCD_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = (int16_t)(2 * err);
        if (e2 >= dy) { err = (int16_t)(err + dy); x0 = (int16_t)(x0 + sx); }
        if (e2 <= dx) { err = (int16_t)(err + dx); y0 = (int16_t)(y0 + sy); }
    }
}

/* 채워진 원 (신호등 점, 사람 머리 등) - 미드포인트 원 알고리즘을
 * 가로 스팬(FillRect) 단위로 채워서 그림.
 * 참고: 반지름이 작을 때(4~5px) DrawCircle(윤곽선)로 그리면 8방향 대칭
 * 특성상 점이 몇 개 안 찍혀서 다이아몬드처럼 보이므로, 신호등 점은
 * 항상 이 FillCircle로 그려서(비활성일 땐 어두운 색) 매끈한 원으로 표시함. */
static void LCD_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t x = r, y = 0, err = 0;
    while (x >= y) {
        LCD_FillRect((uint16_t)(x0 - x), (uint16_t)(y0 + y), (uint16_t)(2 * x + 1), 1, color);
        LCD_FillRect((uint16_t)(x0 - x), (uint16_t)(y0 - y), (uint16_t)(2 * x + 1), 1, color);
        LCD_FillRect((uint16_t)(x0 - y), (uint16_t)(y0 + x), (uint16_t)(2 * y + 1), 1, color);
        LCD_FillRect((uint16_t)(x0 - y), (uint16_t)(y0 - x), (uint16_t)(2 * y + 1), 1, color);
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        if (err > 0)  { x--; err -= 2 * x + 1; }
    }
}

static inline void swap_i16(int16_t *a, int16_t *b) { int16_t t = *a; *a = *b; *b = t; }

/* 화살촉(삼각형) 채우기 - 표준 스캔라인 방식: y를 위에서 아래로 훑으면서
 * 그 y줄에서 삼각형의 왼쪽/오른쪽 경계 x를 계산해 한 줄씩 FillRect로 채움 */
static void LCD_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                                 int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;
    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }
    if (y1 > y2) { swap_i16(&y1, &y2); swap_i16(&x1, &x2); }
    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }

    if (y0 == y2) {
        a = b = x0;
        if (x1 < a) a = x1; else if (x1 > b) b = x1;
        if (x2 < a) a = x2; else if (x2 > b) b = x2;
        LCD_FillRect((uint16_t)a, (uint16_t)y0, (uint16_t)(b - a + 1), 1, color);
        return;
    }

    int16_t dx01 = (int16_t)(x1 - x0), dy01 = (int16_t)(y1 - y0);
    int16_t dx02 = (int16_t)(x2 - x0), dy02 = (int16_t)(y2 - y0);
    int16_t dx12 = (int16_t)(x2 - x1), dy12 = (int16_t)(y2 - y1);
    int32_t sa = 0, sb = 0;

    last = (y1 == y2) ? y1 : (int16_t)(y1 - 1);
    for (y = y0; y <= last; y++) {
        a = (int16_t)(x0 + sa / dy01);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx01; sb += dx02;
        if (a > b) swap_i16(&a, &b);
        LCD_FillRect((uint16_t)a, (uint16_t)y, (uint16_t)(b - a + 1), 1, color);
    }
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = (int16_t)(x1 + sa / dy12);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx12; sb += dx02;
        if (a > b) swap_i16(&a, &b);
        LCD_FillRect((uint16_t)a, (uint16_t)y, (uint16_t)(b - a + 1), 1, color);
    }
}

/* ------------------------------- 4) 5x7 폰트 ------------------------------- *
 * 실제 사용하는 문구(RIGHT TURN / LEFT TURN / STRAIGHT / CAUTION /
 * OPP. TURN / PEDESTRIAN / AI N/A / SEC / 0-9)에 필요한 문자만 최소 구성했습니다.
 * 새 문구가 필요하면 이 표에 글리프를 추가하면 됩니다.
 * 각 문자는 5개의 컬럼(byte)로 구성, bit0=윗줄 ~ bit6=아랫줄(7행).
 * ------------------------------------------------------------------------ */
typedef struct { char ch; uint8_t col[5]; } FontGlyph;

static const FontGlyph font_table[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x60,0x60,0x00}},
    {'/', {0x40,0x20,0x10,0x08,0x04}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}},
    {'3', {0x22,0x41,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
	{'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
};
#define FONT_TABLE_LEN (sizeof(font_table) / sizeof(font_table[0]))

static const FontGlyph *Font_Find(char c)
{
    for (uint8_t i = 0; i < FONT_TABLE_LEN; i++) {
        if (font_table[i].ch == c) return &font_table[i];
    }
    return &font_table[0]; /* 미지원 문자는 공백 처리 */
}

/* size: 1이면 5x7 픽셀 그대로, 2면 픽셀 하나를 2x2 블록으로 확대(카운트다운
 * 숫자처럼 크게 보여주고 싶을 때 사용) */
static void LCD_DrawChar(int16_t x, int16_t y, char c, uint16_t color, uint8_t size)
{
    const FontGlyph *g = Font_Find(c);
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t line = g->col[col];
        for (uint8_t row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                LCD_FillRect((uint16_t)(x + col * size), (uint16_t)(y + row * size),
                                 size, size, color);
            }
        }
    }
}

static void LCD_DrawString(int16_t x, int16_t y, const char *s, uint16_t color, uint8_t size)
{
    int16_t cx = x;
    while (*s) {
        LCD_DrawChar(cx, y, *s, color, size);
        cx = (int16_t)(cx + (5 + 1) * size); /* 글자 폭 5 + 자간 1 */
        s++;
    }
}

/* --------------------------------- 5) 아이콘 -------------------------------- *
 * 전부 위의 도형 프리미티브(FillRect/FillCircle/DrawLine/FillTriangle) 조합으로만
 * 그려서, 이미지 리소스나 별도 비트맵 없이 컴파일 타임에 고정된 벡터 아이콘처럼
 * 동작합니다. */

static void Icon_Car(int16_t cx, int16_t cy, uint16_t color)
{
    LCD_FillRect((uint16_t)(cx - 8), (uint16_t)(cy - 2), 16, 7, color); /* 차체 */
    LCD_FillRect((uint16_t)(cx - 5), (uint16_t)(cy - 7), 10, 5, color); /* 지붕 */
}

static void Icon_Pedestrian(int16_t cx, int16_t cy, uint16_t color)
{
    LCD_FillCircle(cx, (int16_t)(cy - 8), 3, color);                 /* 머리 */
    LCD_DrawLine(cx, (int16_t)(cy - 5), cx, (int16_t)(cy + 3), color); /* 몸통 */
    LCD_DrawLine(cx, (int16_t)(cy + 3), (int16_t)(cx - 4), (int16_t)(cy + 8), color); /* 왼다리 */
    LCD_DrawLine(cx, (int16_t)(cy + 3), (int16_t)(cx + 4), (int16_t)(cy + 8), color); /* 오른다리 */
    LCD_DrawLine(cx, (int16_t)(cy - 3), (int16_t)(cx - 4), cy, color);                /* 왼팔 */
    LCD_DrawLine(cx, (int16_t)(cy - 3), (int16_t)(cx + 4), cy, color);                /* 오른팔 */
}

static void Icon_TrafficLight(int16_t cx, int16_t cy, uint16_t body_color, uint16_t alert_color)
{
    /* 몸체 (각진 사각형, 세로로 긴 형태) */
    LCD_FillRect((uint16_t)(cx - 4), (uint16_t)(cy - 9), 8, 18, body_color);

    /* 점 3개: 위(강조/빨강) - 중간 - 아래, 배경색으로 파서 대비 */
    LCD_FillCircle(cx, (int16_t)(cy - 5), 2, alert_color);
    LCD_FillCircle(cx, cy,               2, COLOR_BG);
    LCD_FillCircle(cx, (int16_t)(cy + 5), 2, COLOR_BG);
}

static void Icon_Warning(int16_t cx, int16_t cy, uint16_t color)
{
    LCD_FillRect((uint16_t)(cx - 2), (uint16_t)(cy - 9), 4, 12, color); /* 막대 */
    LCD_FillCircle(cx, (int16_t)(cy + 7), 2, color);                    /* 점 */
}
/* 좌측 60x100 영역(x:0~60, y:29~128)에 그리는 회전/직진 화살표 3종.
 * 셋 다 같은 박스 안에서 x=30을 기준으로 서로 대칭/변형된 좌표를 씁니다.
 * 우회전: 세로 막대 -> 둥근 모서리 -> 가로 막대 -> 화살촉(오른쪽) 순으로 겹침 */
static void Icon_ArrowRightTurn(uint16_t color)
{
    LCD_FillRect(12, 65, 8, 40, color);
    LCD_FillCircle(16, 65, 5, color);
    LCD_FillRect(16, 61, 24, 8, color);
    LCD_FillTriangle(46, 65, 36, 55, 36, 75, color);
}

/* 좌회전: 우회전 화살표를 x=30 기준으로 좌우 반전한 좌표 (x' = 60 - x) */
static void Icon_ArrowLeftTurn(uint16_t color)
{
    LCD_FillRect(40, 65, 8, 40, color);
    LCD_FillCircle(44, 65, 5, color);
    LCD_FillRect(20, 61, 24, 8, color);
    LCD_FillTriangle(14, 65, 24, 55, 24, 75, color);
}

/* 직진: 세로 막대 + 위쪽을 가리키는 화살촉 */
static void Icon_ArrowStraight(uint16_t color)
{
    LCD_FillRect(26, 65, 8, 40, color);
    LCD_FillTriangle(30, 50, 18, 65, 42, 65, color);
}

/* -------------------------------- 6) 대시보드 -------------------------------- *
 * 이 아래부터가 실제로 화면 레이아웃(좌표)을 정하는 "애플리케이션" 계층입니다.
 * 위의 1~5번은 전부 이 계층을 위한 도구 상자입니다. */


#define COLOR_TEXT       LCD_WHITE
#define COLOR_DIVIDER    0x39C7
#define COLOR_BLUE       0x3D7F
#define COLOR_AMBER      0xFD20
#define COLOR_DANGER     LCD_RED
#define COLOR_ICON       0xC618
#define COLOR_DIM_RED    0x4000
#define COLOR_DIM_AMBER  0x4200
#define COLOR_DIM_GREEN  0x0320

/* 신호색 지속시간(초)과 다음 신호색. main.c는 이 배열을 직접 보지 않고
 * 아래 Dashboard_SignalDuration()/Dashboard_NextSignal() 함수로만 접근합니다. */
//static const uint8_t     signal_duration[SIG_COUNT] = { 5, 3, 15 };  /* RED, GREEN, YELLOW */
//static const SignalColor signal_next[SIG_COUNT]     = { SIG_GREEN, SIG_RED, SIG_YELLOW }; /* 적->녹->황->적 */

//uint8_t Dashboard_SignalDuration(SignalColor color)
//{
//    return signal_duration[color];
//}

//SignalColor Dashboard_NextSignal(SignalColor color)
//{
//    return signal_next[color];
//}

/* 상단 바(y:0~28) 왼쪽에 신호등 3점을 가로로 배치.
 * 비활성 신호도 DrawCircle(윤곽선) 대신 FillCircle(어두운 색)로 그려서
 * 작은 반지름에서 윤곽선이 다이아몬드처럼 보이는 문제를 없앰. */
void Dashboard_DrawSignalDots(SignalColor active)
{
    LCD_FillRect(0, 0, 52, 28, COLOR_BG);

    const int16_t cy = 14;

    LCD_FillCircle(10, cy, 5, (active == SIG_RED)    ? LCD_RED    : COLOR_DIM_RED);
    LCD_FillCircle(26, cy, 5, (active == SIG_YELLOW) ? LCD_YELLOW : COLOR_DIM_AMBER);
    LCD_FillCircle(42, cy, 5, (active == SIG_GREEN)  ? LCD_GREEN  : COLOR_DIM_GREEN);
}

static void Dashboard_DrawCountdown(uint8_t sec)
{
    char buf[3];
    buf[0] = (char)('0' + (sec / 10));
    buf[1] = (char)('0' + (sec % 10));
    buf[2] = '\0';
    LCD_DrawString(55, 3, buf, COLOR_TEXT, 2);
}

void Dashboard_UpdateCountdown(uint8_t sec)
{
    LCD_FillRect(55, 2, 26, 16, COLOR_BG);
    Dashboard_DrawCountdown(sec);
}

/* 상단 바 오른쪽의 주행상태 라벨("RIGHT TURN"/"LEFT TURN"/"STRAIGHT")과
 * 좌측 화살표 아이콘을 dir 하나로 같이 결정해서 그림. */
void Dashboard_DrawDirection(TurnDirection dir)
{
    /* 화살표 영역(좌측 60x99, 상단 바 아래) 초기화 */
    LCD_FillRect(0, 29, 60, (uint16_t)(LCD_H - 29), COLOR_BG);
    /* 라벨 영역(상단 바 오른쪽) 초기화 */
    LCD_FillRect(80, 1, (uint16_t)(LCD_W - 80), 26, COLOR_BG);

    if (dir == DIR_ERROR) return;

    const char *label;
    switch (dir) {
        case DIR_LEFT:
            label = "LEFT TURN";
            Icon_ArrowLeftTurn(COLOR_BLUE);
            break;
        case DIR_RIGHT:
            label = "RIGHT TURN";
            Icon_ArrowRightTurn(COLOR_BLUE);
            break;
        case DIR_STRAIGHT:
        default:
            label = "STRAIGHT";
            Icon_ArrowStraight(COLOR_BLUE);
            break;
    }
    LCD_DrawString(90, 10, label, COLOR_TEXT, 1);
}

/* 우측 경고 한 항목의 표시 내용 (사이드바 색, 아이콘, 라벨/상태 문구, 상태 색) */
typedef struct {
    uint8_t     flag;
    uint16_t    bar_color;
    const char *label;
    const char *status;
    uint16_t    status_color;
    uint8_t     icon; /* 0 = 차량, 1 = 보행자, 2 = 신호등 */
} WarningInfo;

//우회전 시 경고: 보행자 / 좌측 직진 차량 / 대향 비보호좌회전
static const WarningInfo warning_table_right[] = {
    { WARN_STRAIGHT_VEHICLE, COLOR_AMBER,  "L STRAIGHT", "CAUTION", COLOR_AMBER,  0 },
    { WARN_OPPOSITE_TURN,    COLOR_AMBER,  "OPP.LEFT",  "CAUTION", COLOR_AMBER,  0 },
    { WARN_PEDESTRIAN,       COLOR_DANGER, "PEDESTRIAN", NULL,      COLOR_DANGER, 1 },
};
#define WARNING_TABLE_RIGHT_LEN (sizeof(warning_table_right) / sizeof(warning_table_right[0]))

//좌회전 시 경고: 대향 직진 / 대향 우회전
static const WarningInfo warning_table_left[] = {
    { WARN_STRAIGHT_VEHICLE, COLOR_AMBER,  "OPP.STRAIGHT", "CAUTION",  COLOR_AMBER,  0 },
    { WARN_OPPOSITE_TURN,    COLOR_AMBER,  "OPP. RIGHT",   "CAUTION",  COLOR_AMBER,  0 },
    { WARN_TL,               COLOR_DANGER, "SIGNAL",       "RED SOON", COLOR_DANGER, 2 },
};
#define WARNING_TABLE_LEFT_LEN (sizeof(warning_table_left) / sizeof(warning_table_left[0]))

//통신 에러
static const WarningInfo warning_table_error[] = {
    { WARN_COMM_ERROR, COLOR_DANGER, "COMM ERROR", "NO DATA", COLOR_DANGER, 4 },
    { WARN_PEDESTRIAN, COLOR_DANGER, "PEDESTRIAN", NULL,      COLOR_DANGER, 1 },
};

#define WARNING_TABLE_ERROR_LEN (sizeof(warning_table_error) / sizeof(warning_table_error[0]))

void Dashboard_DrawWarnings(TurnDirection dir, uint8_t mask, uint8_t ped_flag)
{
    const WarningInfo *table;
    uint8_t table_len;

    switch (dir) {
        case DIR_RIGHT:
            table     = warning_table_right;
            table_len = WARNING_TABLE_RIGHT_LEN;
            break;
        case DIR_LEFT:
            table     = warning_table_left;
            table_len = WARNING_TABLE_LEFT_LEN;
            break;
        case DIR_ERROR:
            table     = warning_table_error;
            table_len = WARNING_TABLE_ERROR_LEN;
            break;
        case DIR_STRAIGHT:
        default:
            table     = NULL;
            table_len = 0;
            break;
    }

    const WarningInfo *active[3];
    uint8_t count = 0;

    for (uint8_t i = 0; i < table_len; i++) {
        if (mask & table[i].flag) {
            active[count++] = &table[i];
        }
    }

    LCD_FillRect(61, 29, (uint16_t)(LCD_W - 61), (uint16_t)(LCD_H - 29), COLOR_BG);
    if (count == 0) return;

    const uint16_t area_y = 29;
    const uint16_t area_h = (uint16_t)(LCD_H - area_y);
    const uint16_t row_h  = (uint16_t)(area_h / count);

    for (uint8_t i = 0; i < count; i++) {
        uint16_t y0 = (uint16_t)(area_y + i * row_h);
        uint16_t cy = (uint16_t)(y0 + row_h / 2);

        LCD_FillRect(60, (uint16_t)(y0 + 2), 3, (uint16_t)(row_h - 4), active[i]->bar_color);

        if (active[i]->icon == 0)      Icon_Car(74, cy, COLOR_ICON);
        else if (active[i]->icon == 1) Icon_Pedestrian(74, cy, COLOR_ICON);
        else if (active[i]->icon == 2) Icon_TrafficLight(74, cy, COLOR_ICON, COLOR_DANGER);
        else if (active[i]->icon == 4) Icon_Warning(74, cy, COLOR_DANGER);

        LCD_DrawString(86, (uint16_t)(cy - 9), active[i]->label, COLOR_TEXT, 1);

        /* 보행자 항목만 status가 NULL -> ped_flag로 문구/색 결정 */
        const char *status;
        uint16_t    status_color;
        if (active[i]->status != NULL) {
            status       = active[i]->status;
            status_color = active[i]->status_color;
        } else if (ped_flag == 2) {
            status       = "AI N/A";      /* 2: 에러(AI 인식 불가) */
            status_color = COLOR_DANGER;
        } else {
            status       = "DETECTED";    /* 1: 보행자 있음 */
            status_color = COLOR_DANGER;
        }
        LCD_DrawString(86, (uint16_t)(cy + 3), status, status_color, 1);


    }
}
void Dashboard_DrawStatic(TurnDirection dir, uint8_t warning_mask,
                           SignalColor active, uint8_t sec, uint8_t ped_flag)
{
    LCD_FillScreen(COLOR_BG);
    LCD_FillRect(0, 28, LCD_W, 1, COLOR_DIVIDER);
    LCD_FillRect(60, 28, 1, (uint16_t)(LCD_H - 28), COLOR_DIVIDER);

    Dashboard_DrawSignalDots(active);
    Dashboard_DrawCountdown(sec);
    LCD_DrawString(58, 20, "SEC", 0x7BEF, 1);

    Dashboard_DrawDirection(dir);
    Dashboard_DrawWarnings(dir, warning_mask, ped_flag);
}
