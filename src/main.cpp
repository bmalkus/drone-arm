/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stm32f4xx.h>

#include <USART.h>
#include <I2C.h>
#include <utils.h>

#include <cstdio>


USART uart_usb(USART2, 115200, true);
I2C mpu(I2C1);

extern "C" void SysTick_Handler()
{
  return;
}

uint8_t data1[10];
uint8_t data2[10];
uint8_t *reading = data1;

void read_cb()
{
  if (reading == data1)
    reading = data2;
  else
    reading = data1;
}

void cb()
{
  if (reading == data1)
    mpu.read(data1, 2, &read_cb);
  else
    mpu.read(data2, 2, &read_cb);
}

int main(void)
{
  // Use external clock to generate HSE
  SET_BIT(RCC->CR, RCC_CR_HSEBYP);
  SET_BIT(RCC->CR, RCC_CR_HSEON);
  while (!READ_BIT(RCC->CR, RCC_CR_HSERDY))
    ;

  // LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

  // Use HSE as PLL input
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);
  // Configure PLL prescalers
  MODIFY_REG(
    RCC->PLLCFGR,
    RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLQ,
    RCC_PLLCFGR_PLLM_2 | // PLLM = 4
    RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_2 | // PLLN = 180
    RCC_PLLCFGR_PLLR_1 | // PLLR = 2
    RCC_PLLCFGR_PLLQ_3 // PLLQ = 8
  );

  // Enable PLL
  SET_BIT(RCC->CR, RCC_CR_PLLON);
  while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY))
    ;

  // Configure AHB Prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, 0x00 << RCC_CFGR_HPRE_Pos);
  // APB1 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, 0b101 << RCC_CFGR_PPRE1_Pos);
  // APB2 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, 0b100 << RCC_CFGR_PPRE2_Pos);

  // use PLL as system clock source
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function)
  SystemCoreClockUpdate();
  // uint32_t a = SystemCoreClock;
  // ((void)a);

  /* Set systick to 1ms */
  SysTick_Config(SystemCoreClock / 1000);

  // Enable clock for GPIOA
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
  READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); // dummy read, waiting for effect of previous command

  // Enable clock for SYSCFG
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

  // Enable clock for USART
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

  // Enable clock for I2C
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

  // Set pins as output
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5, 0b01 << GPIO_MODER_MODE5_Pos);
  MODIFY_REG(GPIOC->MODER, 0x00, 0b0101010101 << GPIO_MODER_MODE1_Pos);

  // Set PC13 as input with pull-down(up?)
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE13, 0b00 << GPIO_MODER_MODE13_Pos);
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD13, 0b10 << GPIO_PUPDR_PUPD13_Pos);

  // Set USART pins
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, 0b10 << GPIO_MODER_MODE2_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2, 0b01 << GPIO_PUPDR_PUPD2_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2, 0b11 << GPIO_OSPEEDR_OSPEED2_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2, 0b0111 << GPIO_AFRL_AFSEL2_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3, 0b10 << GPIO_MODER_MODE3_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD3, 0b01 << GPIO_PUPDR_PUPD3_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3, 0b11 << GPIO_OSPEEDR_OSPEED3_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3, 0b0111 << GPIO_AFRL_AFSEL3_Pos);

  MODIFY_REG(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13, SYSCFG_EXTICR4_EXTI13_PC);

  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  // char msg[] = "mąka źdźbło\r\n";

  // Set I2C pins
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE8, 0b10 << GPIO_MODER_MODE8_Pos);
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT8);
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD8, 0b01 << GPIO_PUPDR_PUPD8_Pos);
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED8, 0b11 << GPIO_OSPEEDR_OSPEED8_Pos);
  MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL8, 0b0100 << GPIO_AFRH_AFSEL8_Pos);

  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE9, 0b10 << GPIO_MODER_MODE9_Pos);
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT9);
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD9, 0b01 << GPIO_PUPDR_PUPD9_Pos);
  MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED9, 0b11 << GPIO_OSPEEDR_OSPEED9_Pos);
  MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL9, 0b0100 << GPIO_AFRH_AFSEL9_Pos);

  // MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ, 45 << I2C_CR2_FREQ_Pos);
  // MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE, 0x2E << I2C_TRISE_TRISE_Pos);
  // SET_BIT(I2C1->CCR, I2C_CCR_FS);
  // MODIFY_REG(I2C1->CCR, I2C_CCR_CCR, 0xE1 << I2C_CCR_CCR_Pos);

  // SET_BIT(I2C1->CR1, I2C_CR1_PE);
  // READ_BIT(I2C1->CR1, I2C_CR1_PE);

  // SET_BIT(I2C1->CR1, I2C_CR1_START);
  // READ_BIT(I2C1->CR1, I2C_CR1_START);

  uart_usb.init();
  // uart_usb.send('0');

  mpu.init(45, false, 100);
  mpu.set_addr(0b1101000);

  uint8_t data[] = {0x6B, 0b00000011};
  mpu.send(data, 2, true);

  for(;;)
  {
    // while (uart_usb.is_sending())
    //   ;
    // uart_usb.send(READ_BIT(I2C1->SR1, I2C_SR1_SB) + '0');
    // while (uart_usb.is_sending())
    //   ;
    // uart_usb.send(READ_BIT(I2C1->SR1, I2C_SR1_ADDR) + '0');
    // while (uart_usb.is_sending())
    //   ;
    // uart_usb.send("\r\n");

    // if (READ_BIT(I2C1->SR1, I2C_SR1_SB))
    //   I2C1->DR = 0b11010001;

    Delay(30);

//     if(READ_BIT(I2C1->SR1, I2C_SR1_ADDR))
//       I2C1->SR2;

    // Delay(150);

    // uart_usb.send(mpu.send(0x00) + '0');
    // uint8_t r[10] = {0, 0};
    if (!mpu.is_sending())
      mpu.send(0x43, false, &cb);
    char str[100];
    // mpu.read(r, 2);
    sprintf(str, "%d\n", int16_t((int16_t(reading[0]) << 8) | reading[1]));
    uart_usb.send(str);
  }

  // /* Set systick to 1ms */
  // SysTick_Config(100000000 / 1000);

  // /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  // SystemCoreClock = 100000000;
}
