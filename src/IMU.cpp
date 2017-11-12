#include <IMU.h>

IMU::IMU(I2C *i2c):
  _i2c(i2c), _ready_to_read(false)
{
  // empty
}

void IMU::init()
{
  _i2c->init(45, false, 100);
  _i2c->set_addr(0b1101000);

  write_buf[0] = 0x6B;
  write_buf[1] = 0b00000011;

  _i2c->send(write_buf, 2, true, &init_data_sent_handler, this);
}

// void IMU::calibrate(uint8_t probes)
// {
//   _calibrating = probes;
//   while (_calibrating > 0)
//     ;
// }

void IMU::read_all()
{
  if (!_i2c->is_sending())
    _i2c->send(0x43, false, &reg_addr_sent_handler, this);
}

Readings IMU::acc()
{
  return *_curr_acc;
}

Readings IMU::gyro()
{
  return *_curr_gyro;
}

void IMU::gyro_reading_done()
{
  _read_buf_gyro->x = __REV16(_read_buf_gyro->x);
  _read_buf_gyro->y = __REV16(_read_buf_gyro->y);
  _read_buf_gyro->z = __REV16(_read_buf_gyro->z);
  Readings *tmp = _curr_gyro;
  _curr_gyro = _read_buf_gyro;
  _read_buf_gyro = tmp;
  _read_gyro = false;
}

void IMU::acc_reading_done()
{
  _read_buf_acc->x = __REV16(_read_buf_acc->x);
  _read_buf_acc->y = __REV16(_read_buf_acc->y);
  _read_buf_acc->z = __REV16(_read_buf_acc->z);
  Readings *tmp = _curr_acc;
  _curr_acc = _read_buf_acc;
  _read_buf_acc = tmp;
  _read_gyro = true;
}

void reg_addr_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  if (imu->_read_gyro)
    imu->_i2c->read(imu->_read_buf_gyro->data, 6, &gyro_reading_done_handler, imu);
  else
    imu->_i2c->read(imu->_read_buf_acc->data, 6, &acc_reading_done_handler, imu);
}

void gyro_reading_done_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->gyro_reading_done();
  imu->_i2c->send(0x3B, false, &reg_addr_sent_handler, imu);
}

void acc_reading_done_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->acc_reading_done();
}

void init_data_sent_handler(void *_imu)
{
  IMU *imu = (IMU*)_imu;
  imu->_ready_to_read = true;
}
