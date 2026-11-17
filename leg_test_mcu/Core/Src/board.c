#include "board.h"
#include "joints.h"
#include "leg_test_config.h"

TIM_HandleTypeDef htim2, htim3, htim4;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart3;

static void clock_init(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // 8 MHz from the ST-LINK MCO (HSE bypass) -> PLL -> 96 MHz, same as the main firmware
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_BYPASS;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 4;
  osc.PLL.PLLN = 96;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4;
  osc.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    Error_Handler();

  // APB1 = 48 MHz (timers 96 MHz), APB2 = 96 MHz
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

static bool joint_in_leg(uint8_t joint, const uint8_t leg_joints[3])
{
  return joint == leg_joints[0] || joint == leg_joints[1] || joint == leg_joints[2];
}

static void gpio_init(const uint8_t leg_joints[3])
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN, GPIO_PIN_RESET);
  g.Pin = LED_GREEN_PIN | LED_BLUE_PIN | LED_RED_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &g);

  g.Pin = BUTTON_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;  // the board has its own pull-down
  HAL_GPIO_Init(BUTTON_PORT, &g);

  for (uint8_t j = 0; j < JOINT_COUNT; j++)
  {
    g.Pin = JOINTS[j].pwm_pin;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (joint_in_leg(j, leg_joints))
    {
      // Timer output, compare = 0 -> line held low, no pulses until engaged
      g.Mode = GPIO_MODE_AF_PP;
      g.Alternate = JOINTS[j].pwm_af;
    }
    else
    {
      // Servos of the other legs: signal held low, they stay limp
      HAL_GPIO_WritePin(JOINTS[j].pwm_port, JOINTS[j].pwm_pin, GPIO_PIN_RESET);
      g.Mode = GPIO_MODE_OUTPUT_PP;
      g.Alternate = 0;
    }
    HAL_GPIO_Init(JOINTS[j].pwm_port, &g);
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    const joint_info_t *jt = &JOINTS[leg_joints[i]];
    g.Pin = jt->adc_pin;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Alternate = 0;
    HAL_GPIO_Init(jt->adc_port, &g);
  }
}

static void pwm_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
  TIM_OC_InitTypeDef oc = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = PWM_PRESCALER;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = PWM_ARR;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    Error_Handler();

  // Compare preload is enabled by the HAL: a new pulse width starts at the next period, never mid-pulse
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  const uint32_t channels[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
  for (uint8_t c = 0; c < 4; c++)
    if (HAL_TIM_PWM_ConfigChannel(htim, &oc, channels[c]) != HAL_OK)
      Error_Handler();
}

static void pwm_init(const uint8_t leg_joints[3])
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  pwm_timer_init(&htim2, TIM2);
  pwm_timer_init(&htim3, TIM3);
  pwm_timer_init(&htim4, TIM4);

  // Only the leg under test gets its channels started (pulse width still 0)
  for (uint8_t i = 0; i < 3; i++)
  {
    const joint_info_t *jt = &JOINTS[leg_joints[i]];
    if (HAL_TIM_PWM_Start(jt->htim, jt->tim_channel) != HAL_OK)
      Error_Handler();
  }
}

static void adc_init(const uint8_t leg_joints[3])
{
  ADC_ChannelConfTypeDef ch = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_adc1.Instance = DMA2_Stream4;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    Error_Handler();
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  // Rank 1..3 = body, femur, tibia of the leg under test
  for (uint8_t i = 0; i < 3; i++)
  {
    ch.Channel = JOINTS[leg_joints[i]].adc_channel;
    ch.Rank = i + 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK)
      Error_Handler();
  }
}

static void uart_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_USART3_CLK_ENABLE();
  // PD8 = TX, PD9 = RX, wired to the ST-LINK virtual COM port
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &g);

  huart3.Instance = USART3;
  huart3.Init.BaudRate = LOG_BAUDRATE;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void board_init(const uint8_t leg_joints[3])
{
  HAL_Init();
  clock_init();
  gpio_init(leg_joints);
  pwm_init(leg_joints);
  adc_init(leg_joints);
  uart_init();
}

void board_start_feedback(volatile uint16_t *buffer, uint32_t length)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buffer, length) != HAL_OK)
    Error_Handler();
}

void board_led(uint16_t pin, bool on)
{
  HAL_GPIO_WritePin(LED_PORT, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool board_button_raw(void)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET;
}

void board_release_all_pwm(void)
{
  // Pulse width 0 from the next period on: servos limp. Register access only, usable from fault handlers.
  TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0;
  TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; TIM3->CCR4 = 0;
  TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; TIM4->CCR4 = 0;
}

void Error_Handler(void)
{
  __disable_irq();
  board_release_all_pwm();
  LED_PORT->BSRR = LED_RED_PIN;
  LED_PORT->BSRR = (uint32_t)(LED_GREEN_PIN | LED_BLUE_PIN) << 16;
  while (1)
  {
  }
}
