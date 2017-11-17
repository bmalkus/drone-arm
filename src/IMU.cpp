#include <IMU.h>
#include <utils.h>

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

  _i2c->send(_helper_buf, 2, true, &__init_data_sent_handler, this);
}

void IMU::calibrate(uint32_t probes)
{
  int32_t gyro_off[3] = {0, 0, 0};
  int32_t acc_off[3] = {0, 0, 0};
  while (_i2c->is_sending())
    ;
  uint32_t calibration = probes;
  while (calibration > 0)
  {
    read_all();
    while (_read_gyro || _read_acc)
      ;
    Readings g = raw_gyro();
    Readings a = raw_acc();
    for (uint8_t i = 0; i < 3; ++i)
    {
      gyro_off[i] += g.data16[i];
      acc_off[i] += a.data16[i];
    }
    calibration -= 1;
    Delay(2);
  }
  for (uint8_t i = 0; i < 3; ++i)
  {
    gyro_off[i] /= int32_t(probes);
    acc_off[i] /= int32_t(probes);
  }
  _gyro_offset = {{(int16_t) gyro_off[0], (int16_t) gyro_off[1], (int16_t) gyro_off[2]}};
  _acc_offset = {{(int16_t) acc_off[0], (int16_t) acc_off[1], (int16_t) acc_off[2]}};
}

void IMU::read_all(cb_type done_cb, void *user_data, cb_type failed_cb)
{
  if (_i2c->is_sending())
    return;

  _done_cb = done_cb;
  _user_data = user_data;
  _failed_cb = failed_cb;

  _read_gyro = true;
  _read_acc = true;
  _read_next();
}

Readings IMU::acc()
{
  return {{
    (int16_t) clamp(_acc[_curr_acc].x - _acc_offset.x, INT16_MIN, INT16_MAX),
    (int16_t) clamp(_acc[_curr_acc].y - _acc_offset.y, INT16_MIN, INT16_MAX),
    (int16_t) clamp(_acc[_curr_acc].z - _acc_offset.z, INT16_MIN, INT16_MAX)
  }};
}

Readings IMU::gyro()
{
  return {{
    (int16_t) clamp(_gyro[_curr_gyro].x - _gyro_offset.x, INT16_MIN, INT16_MAX),
    (int16_t) clamp(_gyro[_curr_gyro].y - _gyro_offset.y, INT16_MIN, INT16_MAX),
    (int16_t) clamp(_gyro[_curr_gyro].z - _gyro_offset.z, INT16_MIN, INT16_MAX)
  }};
}

Readings IMU::raw_acc()
{
  return _acc[_curr_acc];
}

Readings IMU::raw_gyro()
{
  return _gyro[_curr_gyro];
}

bool IMU::_read_next()
{
  if (_read_gyro)
  {
    _i2c->send(0x43, false, &__reg_addr_sent_handler, this);
    return true;
  }
  else if (_read_acc)
  {
    _i2c->send(0x3B, false, &__reg_addr_sent_handler, this);
    return true;
  }
  return false;
}

void IMU::_reg_addr_sent_handler()
{
  if (_read_gyro)
    _target_buf = &(_gyro[1-_curr_gyro]);
  else if (_read_acc)
    _target_buf = &(_acc[1-_curr_acc]);
  _i2c->read(_target_buf->data, 6, &__reading_done_handler, this);
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
  if (!_read_next() && _done_cb)
    _done_cb(_user_data);
}

void IMU::_init_data_sent_handler()
{
  _ready_to_read = true;
}


void __reg_addr_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_reg_addr_sent_handler();
}

void __reading_done_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_reading_done_handler();
}

void __init_data_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_init_data_sent_handler();
}
