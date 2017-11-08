#include <I2C.h>

#include <utils.h>

I2C::I2C(I2C_TypeDef *I2C):
  _I2C(I2C)
{
  // empty
}

void I2C::init(uint32_t APB_freq_MHz, bool fs, uint8_t I2C_freq_kHz)
{
  CLEAR_BIT(_I2C->CR1, I2C_CR1_PE);
  READ_BIT(_I2C->CR1, I2C_CR1_PE);

  MODIFY_REG(_I2C->CR2, I2C_CR2_FREQ, 45 << I2C_CR2_FREQ_Pos);

  float period = 1000 / APB_freq_MHz;
  uint32_t trise;
  uint32_t CCR;
  if (fs)
  {
    SET_BIT(_I2C->CCR, I2C_CCR_FS);
    SET_BIT(_I2C->CCR, I2C_CCR_DUTY);
    trise = 300 / period;
    CCR = 1 / ((25 * I2C_freq_kHz * period) / 1000000);
  }
  else
  {
    trise = 1000 / period;
    CCR = 1 / ((2 * I2C_freq_kHz * period) / 1000000);
  }
  trise += 1;

  // MODIFY_REG(_I2C->TRISE, I2C_TRISE_TRISE, trise << I2C_TRISE_TRISE_Pos);
  // MODIFY_REG(_I2C->CCR, I2C_CCR_CCR, CCR << I2C_CCR_CCR_Pos);
  MODIFY_REG(_I2C->TRISE, I2C_TRISE_TRISE, 0x2E << I2C_TRISE_TRISE_Pos);
  MODIFY_REG(_I2C->CCR, I2C_CCR_CCR, 0xE1 << I2C_CCR_CCR_Pos);

  SET_BIT(_I2C->CR1, I2C_CR1_PE);
  READ_BIT(_I2C->CR1, I2C_CR1_PE);
}

void I2C::set_addr(uint8_t addr)
{
  _addr = addr << 1;
}

bool I2C::send(uint8_t byte, bool do_stop_cond)
{
  start_cond(WRITE);
  while (!READ_BIT(_I2C->SR1, I2C_SR1_TXE))
    ;
  _I2C->DR = byte;
  while (!READ_BIT(_I2C->SR1, I2C_SR1_BTF))
    ;
  if (do_stop_cond)
    stop_cond();
  return true;
}

bool I2C::send(const uint8_t *bytes, uint8_t len, bool do_stop_cond)
{
  start_cond(WRITE);
  for (int i = 0; i < len; ++i)
  {
    while (!READ_BIT(_I2C->SR1, I2C_SR1_TXE))
      ;
    _I2C->DR = bytes[i];
  }
  while (!READ_BIT(_I2C->SR1, I2C_SR1_BTF))
    ;
  if (do_stop_cond)
    stop_cond();
  return true;
}

bool I2C::read(uint8_t *buf, int len)
{
  start_cond(READ);
  if (len > 1)
  {
    SET_BIT(_I2C->CR1, I2C_CR1_ACK);
  }
  for (int i = 0; i < len; ++i)
  {
    while (!READ_BIT(_I2C->SR1, I2C_SR1_RXNE))
      ;
    if (i == len - 2)
      CLEAR_BIT(_I2C->CR1, I2C_CR1_ACK);
    *buf = _I2C->DR;
    ++buf;
  }
  stop_cond();
  return true;
}

bool I2C::start_cond(RW rw)
{
  SET_BIT(_I2C->CR1, I2C_CR1_START);
  READ_BIT(_I2C->CR1, I2C_CR1_START);

  while (!READ_BIT(_I2C->SR1, I2C_SR1_SB))
    ;

  _I2C->DR = _addr | rw;

  while (!READ_BIT(_I2C->SR1, I2C_SR1_ADDR))
    ;

  I2C1->SR2;
  return true;
}

void I2C::stop_cond()
{
  SET_BIT(_I2C->CR1, I2C_CR1_STOP);
}
