#ifndef IMU_H
#define IMU_H

#include <I2C.h>

union Readings {
  struct {
    volatile int16_t x, y, z;
  };
  volatile uint8_t data[6];
  volatile int16_t data16[3];
};

class IMU
{
public:
  IMU(I2C *i2c);

  void init();

  void calibrate(uint32_t probes);

  void read_all();

  Readings acc();
  Readings gyro();

  Readings true_acc();
  Readings true_gyro();

  bool ready_to_read() { return _ready_to_read; }

  Readings _gyro_offset = {{0, 0, 0}};
  Readings _acc_offset = {{0, 0, 0}};

private:
  I2C *_i2c;

  uint8_t _helper_buf[16];

  volatile bool _ready_to_read = false;

  volatile bool _read_gyro = false;
  volatile bool _read_acc = false;

  Readings _gyro[2];
  Readings _acc[2];
  volatile uint8_t _curr_gyro = 0;
  volatile uint8_t _curr_acc = 0;

  Readings *volatile _target_buf = nullptr;

  void _read_next();

  void _reg_addr_sent_handler();
  void _reading_done_handler();
  void _init_data_sent_handler();

  friend void reg_addr_sent_handler(void *);
  friend void reading_done_handler(void *);
  friend void init_data_sent_handler(void *);
};

void reg_addr_sent_handler(void *);
void reading_done_handler(void *);
void init_data_sent_handler(void *);

#endif /* IMU_H */
