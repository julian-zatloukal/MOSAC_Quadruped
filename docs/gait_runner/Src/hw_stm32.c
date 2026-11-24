/* NUCLEO-F412ZG implementation of hw.h. The peripheral setup is the one that was
 * run on the robot with servo_test_leg (docs/servo_test_leg), as in leg_lab. */
#include "hw.h"
#include "robot.h"
#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* Servo PWM pins, in servo order (TIM2 CH1..4, TIM3 CH1..4, TIM4 CH1..4). */
static const struct { GPIO_TypeDef *port; uint16_t pin; uint8_t af; } PWM_PIN[SERVO_COUNT] = {
    { GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2 },  /*  0 BFR */
    { GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2 },  /*  1 BBR */
    { GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2 },  /*  2 BBL */
    { GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2 },  /*  3 TFR */
    { GPIOC, GPIO_PIN_6,  GPIO_AF2_TIM3 },  /*  4 FFR */
    { GPIOC, GPIO_PIN_7,  GPIO_AF2_TIM3 },  /*  5 BFL */
    { GPIOC, GPIO_PIN_8,  GPIO_AF2_TIM3 },  /*  6 FFL */
    { GPIOC, GPIO_PIN_9,  GPIO_AF2_TIM3 },  /*  7 TFL */
    { GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4 },  /*  8 FBL */
    { GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4 },  /*  9 TBL */
    { GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4 },  /* 10 TBR */
    { GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4 },  /* 11 FBR */
};

#define LED_GREEN_PIN GPIO_PIN_0    /* LD1, PB0  */
#define LED_BLUE_PIN  GPIO_PIN_7    /* LD2, PB7  */
#define LED_RED_PIN   GPIO_PIN_14   /* LD3, PB14 */
#define USER_BTN_PORT GPIOC
#define USER_BTN_PIN  GPIO_PIN_13   /* B1, high when pressed */

static TIM_HandleTypeDef  htim[3];
static UART_HandleTypeDef huart3;
static ADC_HandleTypeDef  hadc1;
static DMA_HandleTypeDef  hdma_adc1;
static const uint32_t     TIM_CH[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static bool               connected[SERVO_COUNT];
static bool               clock_from_hsi;

static volatile uint16_t  adc_buf[HW_ADC_SCANS][HW_ADC_CH];

/* ------------------------------------------------------------ serial --- */

#define TX_BUF_SIZE 8192u
static char              tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head, tx_tail;      /* head: main loop, tail: ISR */
static volatile int16_t  rx_char = -1;

void hw_uart_write(const char *s)
{
    while (*s) {
        uint16_t next = (uint16_t)((tx_head + 1u) % TX_BUF_SIZE);
        if (next == tx_tail) {
            /* Full: wait up to 50 ms for room, then drop the rest. */
            uint32_t t0 = HAL_GetTick();
            USART3->CR1 |= USART_CR1_TXEIE;
            while (next == tx_tail && HAL_GetTick() - t0 < 50u) { }
            if (next == tx_tail) break;
        }
        tx_buf[tx_head] = *s++;
        tx_head = next;
    }
    USART3->CR1 |= USART_CR1_TXEIE;
}

int hw_uart_getc(void)
{
    int c = rx_char;
    rx_char = -1;
    return c;
}

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE))
        rx_char = (int16_t)(uint8_t)USART3->DR;      /* reading DR clears RXNE / ORE */
    if ((USART3->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_tail != tx_head) {
            USART3->DR = (uint8_t)tx_buf[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) % TX_BUF_SIZE);
        } else {
            USART3->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ------------------------------------------------------------- basics --- */

uint32_t hw_millis(void) { return HAL_GetTick(); }

void hw_wait_until(uint32_t t_ms)
{
    while ((int32_t)(HAL_GetTick() - t_ms) < 0)
        __WFI();                        /* SysTick wakes us every 1 ms */
}

bool hw_button_down(void) { return HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN) == GPIO_PIN_SET; }

void hw_led(int led, bool on)
{
    static const uint16_t PIN[3] = { LED_GREEN_PIN, LED_BLUE_PIN, LED_RED_PIN };
    HAL_GPIO_WritePin(GPIOB, PIN[led], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool hw_clock_from_hsi(void) { return clock_from_hsi; }

uint16_t hw_vrefint_cal(void) { return *(const uint16_t *)0x1FFF7A2Au; }   /* VREFINT_CAL_ADDR */

void hw_set_pulse_us(int n, uint16_t us)
{
    if (n < 0 || n >= SERVO_COUNT || !connected[n]) return;
    __HAL_TIM_SET_COMPARE(&htim[n / 4], TIM_CH[n % 4], us);
}

void hw_adc_copy(int ch, int count, uint16_t *dst)
{
    /* NDTR counts down the transfers left in the current pass of the ring. */
    uint32_t written = (uint32_t)(HW_ADC_SCANS * HW_ADC_CH) - __HAL_DMA_GET_COUNTER(&hdma_adc1);
    int s = (int)(written / HW_ADC_CH) - 1;          /* newest complete scan */
    for (int i = count - 1; i >= 0; i--, s--) {
        if (s < 0) s += HW_ADC_SCANS;
        dst[i] = adc_buf[s][ch];
    }
}

/* -------------------------------------------------------- peripherals --- */

static void servo_pins_low(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (int n = 0; n < SERVO_COUNT; n++) {
        HAL_GPIO_WritePin(PWM_PIN[n].port, PWM_PIN[n].pin, GPIO_PIN_RESET);
        g.Pin = PWM_PIN[n].pin;
        HAL_GPIO_Init(PWM_PIN[n].port, &g);
    }
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* 8 MHz MCO from the ST-LINK (HSE bypass) -> PLL -> 96 MHz, as in quadruped_control_mcu. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 4;
    osc.PLL.PLLN       = 96;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 4;
    osc.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /* No MCO clock: internal 16 MHz HSI, same 96 MHz result. */
        osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState            = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM            = 8;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
        clock_from_hsi = true;
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* 48 MHz, timers x2 = 96 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* 96 MHz, ADC clock /4 = 24 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
    g.Pin   = LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin  = USER_BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;                  /* external pull-down on the Nucleo */
    HAL_GPIO_Init(USER_BTN_PORT, &g);
}

static void uart_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;  /* PD8 TX, PD9 RX -> ST-LINK virtual COM port */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

    USART3->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* ADC1 scans the 12 pots and VREFINT continuously; DMA2 Stream4 fills adc_buf as a
 * ring. 13 x (480 + 12) cycles at 24 MHz = 0.27 ms per scan. No interrupts. */
static void adc_init(void)
{
    GPIO_InitTypeDef       g  = {0};
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin  = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin  = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &g);

    hdma_adc1.Instance                 = DMA2_Stream4;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = HW_ADC_CH;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;    /* also >10 us, as VREFINT needs */
    for (int n = 0; n < SERVO_COUNT; n++) {
        ch.Channel = SERVO[n].adc_ch;              /* ADC_CHANNEL_n has the value n */
        ch.Rank    = (uint32_t)n + 1u;
        if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
    }
    ch.Channel = ADC_CHANNEL_VREFINT;              /* also sets TSVREFE */
    ch.Rank    = HW_ADC_VREF + 1u;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, HW_ADC_SCANS * HW_ADC_CH) != HAL_OK) Error_Handler();
    HAL_Delay(30);   /* fill the whole ring once before the first median */
}

/* All three timers run with every compare at 0 (no pulse). The pins are handed
 * to them only if connect_pwm is set. */
static void pwm_init(bool connect_pwm)
{
    static TIM_TypeDef *const INSTANCE[3] = { TIM2, TIM3, TIM4 };
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2u;   /* APB1 prescaler is 2 */
    TIM_OC_InitTypeDef oc = {0};
    GPIO_InitTypeDef   g  = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    for (int t = 0; t < 3; t++) {
        htim[t].Instance               = INSTANCE[t];
        htim[t].Init.Prescaler         = tim_clk / 1000000u - 1u;   /* 1 us tick */
        htim[t].Init.CounterMode       = TIM_COUNTERMODE_UP;
        htim[t].Init.Period            = PWM_PERIOD_US;
        htim[t].Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        htim[t].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_PWM_Init(&htim[t]) != HAL_OK) Error_Handler();
    }

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;                 /* ConfigChannel also enables CCR preload */

    for (int n = 0; n < SERVO_COUNT; n++) {
        if (HAL_TIM_PWM_ConfigChannel(&htim[n / 4], &oc, TIM_CH[n % 4]) != HAL_OK) Error_Handler();
        if (HAL_TIM_PWM_Start(&htim[n / 4], TIM_CH[n % 4]) != HAL_OK) Error_Handler();
    }
    if (!connect_pwm) return;                          /* every pin stays a driven-low output */
    for (int n = 0; n < SERVO_COUNT; n++) {
        g.Pin       = PWM_PIN[n].pin;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_NOPULL;
        g.Speed     = GPIO_SPEED_FREQ_LOW;
        g.Alternate = PWM_PIN[n].af;
        HAL_GPIO_Init(PWM_PIN[n].port, &g);
        connected[n] = true;
    }
}

void hw_init(bool connect_pwm)
{
    HAL_Init();
    servo_pins_low();
    clock_init();
    gpio_init();
    uart_init();
    pwm_init(connect_pwm);
    adc_init();
}

/* ------------------------------------------------------------- faults --- */

/* A fault in the firmware: interrupts off, red LED on, halt. The timers keep
 * sending the last pulses, so the robot holds its pose instead of dropping (cut
 * the servo power, or press RESET, which drops it). Registers only. */
static void __attribute__((noreturn)) fail_safe_halt(void)
{
    __disable_irq();
    GPIOB->BSRR = LED_RED_PIN;
    for (;;) { }
}

void Error_Handler(void)      { fail_safe_halt(); }
void HardFault_Handler(void)  { fail_safe_halt(); }
void MemManage_Handler(void)  { fail_safe_halt(); }
void BusFault_Handler(void)   { fail_safe_halt(); }
void UsageFault_Handler(void) { fail_safe_halt(); }
void SysTick_Handler(void)    { HAL_IncTick(); }
