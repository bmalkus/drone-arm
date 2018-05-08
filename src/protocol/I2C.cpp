#include <protocol/I2C.h>

#include <util/misc.h>
#include <protocol/USART.h>

I2C::I2C(I2C_TypeDef *I2C) :
    _I2C(I2C) {
  // empty
}

void I2C::init(uint32_t /*APB_freq_MHz*/, bool fs, uint8_t /*I2C_freq_kHz*/) {
  // TODO: implement init properly
  CLEAR_BIT(_I2C->CR1, I2C_CR1_PE);
  READ_BIT(_I2C->CR1, I2C_CR1_PE);

  MODIFY_REG(_I2C->CR2, I2C_CR2_FREQ, 45 << I2C_CR2_FREQ_Pos);

  // float period = 1000 / APB_freq_MHz;
  // uint32_t trise;
  // uint32_t CCR;
  if (fs) {
    SET_BIT(_I2C->CCR, I2C_CCR_FS);
    SET_BIT(_I2C->CCR, I2C_CCR_DUTY);
    // trise = 300 / period;
    // CCR = 1 / ((25 * I2C_freq_kHz * period) / 1000000);
  } else {
    // trise = 1000 / period;
    // CCR = 1 / ((2 * I2C_freq_kHz * period) / 1000000);
  }
  // trise += 1;

  // MODIFY_REG(_I2C->TRISE, I2C_TRISE_TRISE, trise << I2C_TRISE_TRISE_Pos);
  // MODIFY_REG(_I2C->CCR, I2C_CCR_CCR, CCR << I2C_CCR_CCR_Pos);
  MODIFY_REG(_I2C->TRISE, I2C_TRISE_TRISE, 0x2E << I2C_TRISE_TRISE_Pos);
  MODIFY_REG(_I2C->CCR, I2C_CCR_CCR, 0xE1 << I2C_CCR_CCR_Pos);

  SET_BIT(_I2C->CR1, I2C_CR1_PE);
  READ_BIT(_I2C->CR1, I2C_CR1_PE);

  // TODO: should be generic
  HandlerHelper::set_handler(HandlerHelper::I2C1_EV_INT, __handle_i2c_event, this);
}

void I2C::set_addr(uint8_t addr) {
  _slave_addr = addr << 1;
}

void I2C::handle_event(HandlerHelper::InterruptType itype) {
  if (itype == HandlerHelper::I2C1_EV_INT) {
    if (READ_BIT(_I2C->SR1, I2C_SR1_SB))
      _I2C->DR = _slave_addr | _current_rw;
    else if (READ_BIT(_I2C->SR1, I2C_SR1_ADDR)) {
      _I2C->SR2;
      if (_current_rw == READ && _len > 1)
        SET_BIT(_I2C->CR1, I2C_CR1_ACK);
    } else if (_current_rw == WRITE) {
      if (READ_BIT(_I2C->SR1, I2C_SR1_TXE)) {
        if (_len > 0) {
          _len -= 1;
          _I2C->DR = _to_send == nullptr ? _byte_to_send : *_to_send++;
          return;
        }
      }
      if (READ_BIT(_I2C->SR1, I2C_SR1_BTF)) {
        MODIFY_REG(_I2C->CR2, I2C_CR2_ITBUFEN_Msk | I2C_CR2_ITEVTEN_Msk, 0x0);
        if (_do_stop_cond)
          stop_cond();
        if (_done_cb != nullptr)
          _done_cb(_user_data);
      }
    } else if (_current_rw == READ) {
      if (READ_BIT(_I2C->SR1, I2C_SR1_RXNE)) {
        if (_len == 2)
          CLEAR_BIT(_I2C->CR1, I2C_CR1_ACK);
        if (_len > 0) {
          _len -= 1;
          *_read_buf++ = _I2C->DR;
        }
        if (_len == 0) {
          MODIFY_REG(_I2C->CR2, I2C_CR2_ITBUFEN_Msk | I2C_CR2_ITEVTEN_Msk, 0x0);
          stop_cond();
          if (_done_cb != nullptr)
            _done_cb(_user_data);
        }
      }
    }
  }
}

bool I2C::send(uint8_t byte, bool do_stop_cond, cb_type cb, void *user_data) {
  start_cond(WRITE);
  _to_send = nullptr;
  _byte_to_send = byte;
  _len = 1;
  _do_stop_cond = do_stop_cond;
  _done_cb = cb;
  _user_data = user_data;
  return true;
}

bool I2C::send(const uint8_t *bytes, uint8_t len, bool do_stop_cond, cb_type cb, void *user_data) {
  start_cond(WRITE);
  _to_send = bytes;
  _len = len;
  _do_stop_cond = do_stop_cond;
  _done_cb = cb;
  _user_data = user_data;
  return true;
}

bool I2C::read(volatile uint8_t *buf, int len, cb_type cb, void *user_data) {
  start_cond(READ);
  _read_buf = buf;
  _len = len;
  _done_cb = cb;
  _user_data = user_data;
  return true;
}

void I2C::start_cond(RW rw) {
  _current_rw = rw;
  SET_BIT(_I2C->CR2, I2C_CR2_ITBUFEN);
  SET_BIT(_I2C->CR2, I2C_CR2_ITEVTEN);
  SET_BIT(_I2C->CR1, I2C_CR1_START);
}

void I2C::abort_sending() {
  MODIFY_REG(_I2C->CR2, I2C_CR2_ITBUFEN_Msk | I2C_CR2_ITEVTEN_Msk, 0x0);
  _len = 0;
  stop_cond();
}

void I2C::stop_cond() {
  SET_BIT(_I2C->CR1, I2C_CR1_STOP);
}

void __handle_i2c_event(HandlerHelper::InterruptType itype, void *_i2c) {
  I2C *i2c = (I2C *) _i2c;
  i2c->handle_event(itype);
}
