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

  void handle_event() { }

  bool send(uint8_t byte, bool do_stop_cond=true);
  bool send(const uint8_t *bytes, uint8_t len, bool do_stop_cond=true);

  bool read(uint8_t *buf, int len);

private:
  I2C_TypeDef *_I2C;
  uint8_t _addr;

  enum RW
  {
    WRITE = 0,
    READ = 1
  };

  bool start_cond(RW rw);
  void stop_cond();
};

#endif /* I2C_H */
