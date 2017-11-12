#ifndef IMU_H
#define IMU_H

#include <I2C.h>

union Readings {
  struct {
    volatile int16_t x, y, z;
  };
  volatile uint8_t data[6];
};

class IMU
{
public:
  IMU(I2C *i2c);

  void init();

  // void calibrate(uint8_t probes);

  void read_all();

  Readings acc();
  Readings gyro();

  bool ready_to_read() { return _ready_to_read; }

private:
  I2C *_i2c;

  uint8_t write_buf[64];
  volatile uint8_t _calibrating = 0;

  volatile bool _ready_to_read;

  volatile bool _read_gyro = true;

  Readings _gyro[2];
  Readings _acc[2];
  Readings * volatile _curr_gyro = _gyro;
  Readings * volatile _curr_acc = _acc;
  Readings * volatile _read_buf_gyro = &(_gyro[1]);
  Readings * volatile _read_buf_acc = &(_acc[1]);

  void gyro_reading_done();
  void acc_reading_done();

  friend void reg_addr_sent_handler(void *);
  friend void acc_reading_done_handler(void *);
  friend void gyro_reading_done_handler(void *);
  friend void init_data_sent_handler(void *);
};

void reg_addr_sent_handler(void *);
void acc_reading_done_handler(void *);
void gyro_reading_done_handler(void *);
void init_data_sent_handler(void *);

#endif /* IMU_H */
