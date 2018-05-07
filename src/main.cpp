#include <cstdio>
#include <cmath>
#include <initializer_list>

#include <stm32f4xx.h>

#include <util/Context.h>
#include <protocol/I2C.h>
#include <IOwrapper/IMU.h>
#include <IOwrapper/Motor.h>
#include <protocol/PWM.h>
#include <protocol/PWMInput.h>
#include <filter/GyroRateFilter.h>
#include <PID/RatePID.h>
#include <util/Timer.h>
#include <protocol/USART.h>
#include <util/USARTHelper.h>
#include <util/misc.h>
#include <IOwrapper/StickInputs.h>
#include <filter/GyroAngleFilter.h>
#include "PID/AnglePID.h"

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
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, 0b00 << RCC_CFGR_HPRE_Pos);
  // APB1 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, 0b101 << RCC_CFGR_PPRE1_Pos);
  // APB2 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, 0b100 << RCC_CFGR_PPRE2_Pos);

  // use PLL as system clock source
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // Update SystemCoreClock to 180 MHz
  SystemCoreClockUpdate();

  // Set systick to 1ms
  SysTick_Config(SystemCoreClock / 1000);

  // Enable clock for GPIOA, GPIOB, GPIOC
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
  RCC->AHB1ENR; // dummy read, waiting for effect of previous command

  // Enable clock for SYSCFG
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

  // Enable clock for USART2
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

  // Enable clock for USART3
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);

  // Enable clock for I2C
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

  // Enable clock for TIM1
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
  READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);

  // Enable clock for TIM2
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);

  // Enable clock for TIM3
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

  // Enable clock for TIM4
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);

  // Enable clock for TIM5
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);

  // Enable clock for TIM6
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
  READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);

  // Enable clock for TIM8
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
  READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);

  // Set PA5 as output (LED)
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE5, 0b01 << GPIO_MODER_MODE5_Pos);

  // Set PC13 as input with pull-up (button)
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE13, 0b00 << GPIO_MODER_MODE13_Pos);
  // MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD13, 0b01 << GPIO_PUPDR_PUPD13_Pos);

  // Set USART2 pins
  // PA2 PA3
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, 0b10 << GPIO_MODER_MODE2_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2, 0b01 << GPIO_PUPDR_PUPD2_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2, 0b11 << GPIO_OSPEEDR_OSPEED2_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2, 0b0111 << GPIO_AFRL_AFSEL2_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3, 0b10 << GPIO_MODER_MODE3_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD3, 0b01 << GPIO_PUPDR_PUPD3_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3, 0b11 << GPIO_OSPEEDR_OSPEED3_Pos);
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3, 0b0111 << GPIO_AFRL_AFSEL3_Pos);

  // Set USART3 pins
  // PC10 PC11
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE10, 0b10 << GPIO_MODER_MODE10_Pos);
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD10, 0b01 << GPIO_PUPDR_PUPD10_Pos);
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED10, 0b11 << GPIO_OSPEEDR_OSPEED10_Pos);
  MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL10, 0b0111 << GPIO_AFRH_AFSEL10_Pos);

  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE11, 0b10 << GPIO_MODER_MODE11_Pos);
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD11, 0b01 << GPIO_PUPDR_PUPD11_Pos);
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED11, 0b11 << GPIO_OSPEEDR_OSPEED11_Pos);
  MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL11, 0b0111 << GPIO_AFRH_AFSEL11_Pos);

  // MODIFY_REG(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13, SYSCFG_EXTICR4_EXTI13_PC);

  // Enable NVIC interrupts
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(TIM4_IRQn);
  NVIC_EnableIRQ(TIM5_IRQn);
  NVIC_EnableIRQ(TIM8_CC_IRQn);

  // Set I2C pins
  // PB8 PB9
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

  // set pins for PWM
  // PA8, PA9, PA10, PA11
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE8, 0b10 << GPIO_MODER_MODE8_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, 0b0001 << GPIO_AFRH_AFSEL8_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9, 0b10 << GPIO_MODER_MODE9_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9, 0b0001 << GPIO_AFRH_AFSEL9_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE10, 0b10 << GPIO_MODER_MODE10_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL10, 0b0001 << GPIO_AFRH_AFSEL10_Pos);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE11, 0b10 << GPIO_MODER_MODE11_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL11, 0b0001 << GPIO_AFRH_AFSEL11_Pos);

  // set pins for PWM input
  // PC6, PC7, PC8, PC9
  // TIM3
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6, 0b10 << GPIO_MODER_MODE6_Pos);
  MODIFY_REG(GPIOC->AFR[0], GPIO_AFRL_AFSEL6, 0b0010 << GPIO_AFRL_AFSEL6_Pos);

  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE7, 0b10 << GPIO_MODER_MODE7_Pos);
  MODIFY_REG(GPIOC->AFR[0], GPIO_AFRL_AFSEL7, 0b0010 << GPIO_AFRL_AFSEL7_Pos);

  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE8, 0b10 << GPIO_MODER_MODE8_Pos);
  MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL8, 0b0010 << GPIO_AFRH_AFSEL8_Pos);

  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE9, 0b10 << GPIO_MODER_MODE9_Pos);
  MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL9, 0b0010 << GPIO_AFRH_AFSEL9_Pos);

  Timer::init();

  USART uart_usb(USART2, 115200);
  USART uart_bt(USART3, 115200);
  uart_usb.init();
  uart_bt.init();

  USARTHelper helper(&uart_bt);

  I2C mpu(I2C1);
  IMU imu(&mpu);
  imu.init();

  PWM pwm(TIM1, 200, 5000, 4);

  PWMInput main_inputs(TIM3, 1'000'000, 4);

  Timer _imu_init(300);

  while (!imu.ready_to_read()) {
    if (!_imu_init) {
      imu.init();
      _imu_init.restart();
    }
  }

  constexpr auto gyro_sensitivity = static_cast<float>((250.f * M_PI)/(180.f * ((1 << 15) - 1)));

  GyroAngleFilter gyro_filter(gyro_sensitivity, 1e-3);

  pwm.init();
  pwm.start();

  main_inputs.init();
  main_inputs.start();
  StickInputs stick_inputs(&main_inputs);

  Motor motors[4] = {
      Motor(&pwm, 1, {{1, 1, 0}}),
      Motor(&pwm, 2, {{-1, -1, 0}}),
      Motor(&pwm, 3, {{-1, 1, 0}}),
      Motor(&pwm, 4, {{1, -1, 0}}),
  };

  for (Motor &m : motors)
    m.init();

  for (Motor &m : motors)
    while(m.ready())
      ;

  AnglePID pid;

  Readings gyro;
  EulerianAngles angles;
  Controls controls;

  Sticks inputs;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-stack-address"
  Context::uart_usb = &uart_usb;
  Context::uart_bt = &uart_bt;
  Context::usart_helper = &helper;
  Context::readings = &gyro;
  Context::eulerian_angles = &angles;
  Context::controls = &controls;
  Context::inputs = &inputs;
  Context::pid = &pid;
  for (int i = 0; i < 4; ++i)
    Context::motors[i] = &motors[i];
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

  Timer loop_timer(0, 999);

  motors[2].disable();
  motors[3].disable();

  for(;;)
  {
    imu.read_all();

    inputs = stick_inputs.get();

    if (stick_inputs.should_be_armed() && !motors[0].armed()) {
      helper.send("Arming\n");
      for (Motor &m : motors)
        m.arm();
    } else if (!stick_inputs.should_be_armed() && motors[0].armed()) {
      helper.send("Disarming\n");
      for (Motor &m : motors)
        m.disarm();
    }

    if (stick_inputs.should_calibrate()) {
      helper.send("Calibrating...");
      imu.calibrate(100);
      gyro_filter.reset_angles();
      helper.send("done\n");
    }

    // wait for sensors data to be read
    // while (imu.is_busy())
    //   ;

    gyro = imu.gyro();

    gyro_filter.set(gyro);

    angles = gyro_filter.get_angles();

    controls = pid.process(angles, inputs);

    for (Motor &m : motors)
      m.set(controls);

    helper.next_loop_iter();

    while(loop_timer)
      ;
    loop_timer.restart();
  }
#pragma clang diagnostic pop
}
