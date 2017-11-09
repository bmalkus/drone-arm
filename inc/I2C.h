#ifndef I2C_H
#define I2C_H

#include <cstdint>

#include <stm32f4xx.h>

#include <HandlerHelper.h>

class I2C : public HandlerClass
{
public:
  I2C(I2C_TypeDef *I2C);

  void init(uint32_t APB_freq_MHz, bool fs, uint8_t duty);

  void set_addr(uint8_t addr);

  virtual void handle_event(HandlerHelper::InterruptType itype) override;

  bool send(uint8_t byte, bool do_stop_cond=true, void (*done_cb)()=nullptr);
  bool send(const uint8_t *bytes, uint8_t len, bool do_stop_cond=true, void (*done_cb)()=nullptr);

  bool read(uint8_t *buf, int len, void (*done_cb)()=nullptr);

  bool is_sending() { return READ_BIT(_I2C->CR2, I2C_CR2_ITBUFEN); }

private:
  enum RW
  {
    WRITE = 0,
    READ = 1
  };

  I2C_TypeDef *_I2C;
  uint8_t _slave_addr;
  RW _current_rw;

  volatile const uint8_t *_to_send;
  uint8_t _byte_to_send;
  volatile uint8_t _len;
  bool _do_stop_cond;
  void (*_done_cb)();

  uint8_t *_read_buf;

  void start_cond(RW rw);
  void stop_cond();
};

#endif /* I2C_H */
