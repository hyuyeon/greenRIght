#include "buzzer.h"
//#include "stm32f429xx.h"

/*
 * CubeMX PLL Clock ê°€??
 *
 * SYSCLK = 180MHz
 * APB1 Timer Clock = 90MHz
 *
 * TIM7 tick = 1us
 */

#define APB1_TIMER_CLK_HZ   84000000UL
#define TIM7_TICK_HZ        1000000UL
#define TIM7_PSC_VALUE      ((APB1_TIMER_CLK_HZ / TIM7_TICK_HZ) - 1)

typedef struct
{
    uint32_t freq_hz;
    uint32_t duration_ms;
} Tone_t;

/*
 * ?„ë£Œ?? ?’ì? ??- ?’ì? ë¯?- ?’ì? ??- ???’ì? ??
 * C6 - E6 - G6 - C7
 */
static const Tone_t exit_melody[] =
{
    {1047, 150},
    {1319, 150},
    {1568, 150},
    {2093, 250},
};

#define EXIT_MELODY_LEN  (sizeof(exit_melody) / sizeof(exit_melody[0]))

volatile uint8_t buzzer_busy = 0;
volatile uint8_t melody_playing = 0;

static volatile uint8_t melody_index = 0;
static volatile uint32_t buzzer_toggle_count = 0;
static volatile uint32_t buzzer_target_toggle_count = 0;

/* ?´ë? ?¨ìˆ˜ */
static void Buzzer_StartTone_Force(uint32_t freq_hz, uint32_t duration_ms);

/* =========================
 * Buzzer GPIO Init
 * PC0 -> GPIO Output
 * ========================= */
void Buzzer_GPIO_Init(void)
{
    // GPIOC clock enable
    RCC->AHB1ENR |= (0x1 << 2);

    // PC0 Output mode: 01
    GPIOC->MODER &= ~(0x3 << (0 * 2));
    GPIOC->MODER |=  (0x1 << (0 * 2));

    // PC0 Push-pull
    GPIOC->OTYPER &= ~(0x1 << 0);

    // PC0 No pull-up / No pull-down
    GPIOC->PUPDR &= ~(0x3 << (0 * 2));

    // PC0 LOW
    GPIOC->BSRR = (0x1 << (0 + 16));
}

/* =========================
 * TIM7 Buzzer Init
 * TIM7 tick = 1us
 * ========================= */
void TIM7_Buzzer_Init(void)
{
    // TIM7 clock enable
    RCC->APB1ENR |= (0x1 << 5);

    /*
     * APB1 Timer Clock = 84MHz
     * PSC = 90 - 1
     * 84MHz / 84 = 1MHz
     * TIM7 tick = 1us
     */
    TIM7->PSC = TIM7_PSC_VALUE;

    // ì´ˆê¸° ARR
    TIM7->ARR = 1000 - 1;

    // Update interrupt enable
    TIM7->DIER |= (0x1 << 0);

    // Update event
    TIM7->EGR |= (0x1 << 0);

    // UIF clear
    TIM7->SR &= ~(0x1 << 0);

    // NVIC TIM7 interrupt enable
    NVIC_SetPriority(TIM7_IRQn, 7U);
    NVIC_EnableIRQ(TIM7_IRQn);

    // ì²˜ìŒ?ëŠ” TIM7 ?•ì?
    TIM7->CR1 &= ~(0x1 << 0);
}

/* =========================
 * Buzzer Stop
 * ========================= */
void Buzzer_Stop(void)
{
    // TIM7 stop
    TIM7->CR1 &= ~(0x1 << 0);

    // TIM7 update flag clear
    TIM7->SR &= ~(0x1 << 0);

    // PC0 LOW
    GPIOC->BSRR = (0x1 << (0 + 16));

    buzzer_busy = 0;
    buzzer_toggle_count = 0;
    buzzer_target_toggle_count = 0;
}

/* =========================
 * ?´ë???Tone Start
 *
 * ?¬ìƒ ì¤??¬ë??€ ?ê??†ì´ ?????œìž‘
 * ë©œë¡œ?”ì—???¤ìŒ ?Œìœ¼ë¡??˜ì–´ê°????¬ìš©
 * ========================= */
static void Buzzer_StartTone_Force(uint32_t freq_hz, uint32_t duration_ms)
{
    uint32_t half_period_us;

    if (freq_hz == 0 || duration_ms == 0)
    {
        Buzzer_Stop();
        return;
    }

    // ?¤ìŠµ??ì£¼íŒŒ???œí•œ
    if (freq_hz < 50)
    {
        freq_hz = 50;
    }

    if (freq_hz > 5000)
    {
        freq_hz = 5000;
    }

    /*
     * TIM7 tick = 1us
     *
     * ?? 1000Hz
     * ?„ì²´ ì£¼ê¸° = 1000us
     * ë°˜ì£¼ê¸?= 500us
     */
    half_period_us = 1000000UL / (freq_hz * 2UL);

    if (half_period_us == 0)
    {
        half_period_us = 1;
    }

    // TIM7 ?•ì? ???¤ì •
    TIM7->CR1 &= ~(0x1 << 0);

    // ë°˜ì£¼ê¸°ë§ˆ??interrupt
    TIM7->ARR = half_period_us - 1;
    TIM7->CNT = 0;

    /*
     * duration_ms ?™ì•ˆ ?„ìš”??toggle ?Ÿìˆ˜
     *
     * 1ì´??™ì•ˆ toggle ??= freq_hz * 2
     * duration_ms ?™ì•ˆ toggle ??= freq_hz * 2 * duration_ms / 1000
     */
    buzzer_toggle_count = 0;
    buzzer_target_toggle_count = (freq_hz * 2UL * duration_ms) / 1000UL;

    if (buzzer_target_toggle_count == 0)
    {
        buzzer_target_toggle_count = 1;
    }

    // ?¤ì • ë°˜ì˜
    TIM7->EGR |= (0x1 << 0);

    // EGRë¡??ê¸´ UIF clear
    TIM7->SR &= ~(0x1 << 0);

    buzzer_busy = 1;

    // TIM7 start
    TIM7->CR1 |= (0x1 << 0);
}

/* =========================
 * ?¸ë? ?¸ì¶œ???¨ì¼ ??ì¶œë ¥
 *
 * ?´ë? ?¬ìƒ ì¤‘ì´ë©?ë¬´ì‹œ
 * ë°˜ë³µ ?¸ì¶œ ì§€ì§€ì§?ë°©ì?
 * ========================= */
void Buzzer_Play_ms(uint32_t freq_hz, uint32_t duration_ms)
{
    if (buzzer_busy || melody_playing)
    {
        return;
    }

    Buzzer_StartTone_Force(freq_hz, duration_ms);
}

/* =========================
 * ì¢…ë£Œ???œìž‘
 *
 * non-blocking
 * ?¸ì¶œ ??ë°”ë¡œ ë¦¬í„´
 * ========================= */
void Buzzer_StartExitMelody(void)
{
    if (buzzer_busy || melody_playing)
    {
        return;
    }

    melody_playing = 1;
    melody_index = 0;

    Buzzer_StartTone_Force(exit_melody[0].freq_hz,
                           exit_melody[0].duration_ms);
}

/* =========================
 * TIM7 IRQ ?´ë? ì²˜ë¦¬ ?¨ìˆ˜
 *
 * stm32f4xx_it.c??TIM7_IRQHandler?ì„œ ?¸ì¶œ
 * ========================= */
void TIM7_Buzzer_IRQHandler(void)
{
    if (TIM7->SR & (0x1 << 0))
    {
        // UIF clear
        TIM7->SR &= ~(0x1 << 0);

        if (buzzer_busy)
        {
            // PC0 toggle
            GPIOC->ODR ^= (0x1 << 0);

            buzzer_toggle_count++;

            if (buzzer_toggle_count >= buzzer_target_toggle_count)
            {
                // ?„ìž¬ ??ì¢…ë£Œ
                TIM7->CR1 &= ~(0x1 << 0);
                GPIOC->BSRR = (0x1 << (0 + 16));

                buzzer_busy = 0;
                buzzer_toggle_count = 0;
                buzzer_target_toggle_count = 0;

                /*
                 * ë©œë¡œ???¬ìƒ ì¤‘ì´ë©??¤ìŒ ?Œìœ¼ë¡??´ë™
                 */
                if (melody_playing)
                {
                    melody_index++;

                    if (melody_index < EXIT_MELODY_LEN)
                    {
                        Buzzer_StartTone_Force(exit_melody[melody_index].freq_hz,
                                               exit_melody[melody_index].duration_ms);
                    }
                    else
                    {
                        melody_playing = 0;
                        Buzzer_Stop();
                    }
                }
            }
        }
        else
        {
            GPIOC->BSRR = (0x1 << (0 + 16));
        }
    }
}

/* =========================
 * System Exit
 *
 * ì¢…ë£Œ??non-blocking ?œìž‘
 * ========================= */
void System_Exit(void)
{
    /*
     * ?¬ê¸°???¤ë¥¸ ì¢…ë£Œ ì²˜ë¦¬ ê°€??
     * ?? LED ?„ê¸°, ëª¨í„° ?•ì?, ?íƒœ ë³€??ë³€ê²???
     */

    Buzzer_StartExitMelody();
}

/* =========================
 * Buzzer DeInit
 *
 * ?„ì „??TIM7 clockê¹Œì? ??
 * ?¤ì‹œ ?°ë ¤ë©?TIM7_Buzzer_Init() ?¬í˜¸ì¶??„ìš”
 * ========================= */
void Buzzer_DeInit(void)
{
    Buzzer_Stop();

    TIM7->DIER &= ~(0x1 << 0);
    NVIC_DisableIRQ(TIM7_IRQn);

    TIM7->CR1 &= ~(0x1 << 0);
    TIM7->SR  &= ~(0x1 << 0);

    GPIOC->BSRR = (0x1 << (0 + 16));

    RCC->APB1ENR &= ~(0x1 << 5);
}
