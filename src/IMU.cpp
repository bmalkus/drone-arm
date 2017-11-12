#include <IMU.h>

IMU::IMU(I2C *i2c):
  _i2c(i2c)
{
  // empty
}

void IMU::init()
{
  _i2c->init(45, false, 100);
  _i2c->set_addr(0b1101000);

  _helper_buf[0] = 0x6B;
  _helper_buf[1] = 0b00000011;

  _i2c->send(_helper_buf, 2, true, &init_data_sent_handler, this);
}

// void IMU::calibrate(uint8_t probes)
// {
//   _calibrating = probes;
//   while (_calibrating > 0)
//     ;
// }

void IMU::read_all()
{
  if (_i2c->is_sending())
    return;
  _read_gyro = true;
  _read_acc = true;
  read_next();
}

Readings IMU::acc()
{
  return _acc[_curr_acc];
}

Readings IMU::gyro()
{
  return _gyro[_curr_gyro];
}

void IMU::read_next()
{
  if (_read_gyro)
    _i2c->send(0x43, false, &reg_addr_sent_handler, this);
  else if (_read_acc)
    _i2c->send(0x3B, false, &reg_addr_sent_handler, this);
}

void IMU::_reg_addr_sent_handler()
{
  if (_read_gyro)
    _target_buf = &(_gyro[1-_curr_gyro]);
  else if (_read_acc)
    _target_buf = &(_acc[1-_curr_acc]);
  _i2c->read(_target_buf->data, 6, &reading_done_handler, this);
}

void IMU::_reading_done_handler()
{
  _target_buf->x = __REV16(_target_buf->x);
  _target_buf->y = __REV16(_target_buf->y);
  _target_buf->z = __REV16(_target_buf->z);
  if (_read_gyro)
  {
    _curr_gyro = (_curr_gyro + 1) % 2;
    _read_gyro = false;
  }
  else if (_read_acc)
  {
    _curr_acc = (_curr_acc + 1) % 2;
    _read_acc = false;
  }
  read_next();
}

void IMU::_init_data_sent_handler()
{
  _ready_to_read = true;
}


void reg_addr_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_reg_addr_sent_handler();
}

void reading_done_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_reading_done_handler();
}

void init_data_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_init_data_sent_handler();
}
