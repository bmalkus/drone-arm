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
#include <IMU.h>
#include <utils.h>

#include <cstdio>


USART uart_usb(USART2, 115200, true);
I2C mpu(I2C1);

extern "C" void SysTick_Handler()
{
  return;
}

int main(void)
{
  // Use external clock to generate HSE
  SET_BIT(RCC->CR, RCC_CR_HSEBYP);
  SET_BIT(RCC->CR, RCC_CR_HSEON);
  while (!READ_BIT(RCC->CR, RCC_CR_HSERDY))
    ;

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

  // Enable clock for GPIOA, GPIOB, GPIOC
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
  RCC->AHB1ENR; // dummy read, waiting for effect of previous command

  // Enable clock for SYSCFG
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

  // Enable clock for USART
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

  // Enable clock for I2C
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

  // Set PA5 as output
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5, 0b01 << GPIO_MODER_MODE5_Pos);

  // Set PC13 as input with pull-up
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE13, 0b00 << GPIO_MODER_MODE13_Pos);
  // MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD13, 0b01 << GPIO_PUPDR_PUPD13_Pos);

  // Set USART pins
  // PA2 PA3
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, 0b10 << GPIO_MODER_MODE2_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2, 0b01 << GPIO_PUPDR_PUPD2_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2, 0b11 << GPIO_OSPEEDR_OSPEED2_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2, 0b0111 << GPIO_AFRL_AFSEL2_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3, 0b10 << GPIO_MODER_MODE3_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD3, 0b01 << GPIO_PUPDR_PUPD3_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3, 0b11 << GPIO_OSPEEDR_OSPEED3_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3, 0b0111 << GPIO_AFRL_AFSEL3_Pos);

  // MODIFY_REG(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13, SYSCFG_EXTICR4_EXTI13_PC);

  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  // Set I2C pins
  // PB8 PA9
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

  IMU imu(&mpu);

  uart_usb.init();

  imu.init();

  Delay(30);

  for(;;)
  {
    Delay(30);

    imu.read_all();

    Readings gyro = imu.gyro();
    Readings acc = imu.acc();

    char str[128];
    sprintf(str, "gyro: %6d %6d %6d     acc:  %6d %6d %6d\n", gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
    uart_usb.send(str);

    if (READ_BIT(GPIOC->IDR, GPIO_IDR_ID13) == 0)
    {
      imu.calibrate(50);
      // Readings gyro = imu._gyro_offset;
      // Readings acc = imu._acc_offset;
      // sprintf(str, "gyro: %6d %6d %6d     acc:  %6d %6d %6d OFFSETS\n", gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
      // uart_usb.send(str);
      // Delay(5000);
    }
  }

  // /* Set systick to 1ms */
  // SysTick_Config(100000000 / 1000);

  // /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  // SystemCoreClock = 100000000;
}
