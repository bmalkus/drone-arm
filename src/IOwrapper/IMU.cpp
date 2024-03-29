#include <IOwrapper/IMU.h>
#include <util/misc.h>
#include <util/Timer.h>
#include <cstdio>
#include <cmath>

IMU::IMU(I2C *i2c) :
    _i2c(i2c) {
  // empty
}

void IMU::init() {
  _i2c->init(45, false, 100);
  _i2c->set_addr(0b1101000);

  _send_buf[0] = {0x1A, {0b00000011, 0b00010000}, 2};
  _send_buf[1] = {0x6B, {0b00000011}, 1};

  _chunks = 2;
  _current = 0;

  _send_from_buf(&__init_data_sent_handler, this);
}

void IMU::_send_from_buf(cb_type cb, void *user_data) {
  _done_cb = cb;
  _cb_user_data = user_data;
  _send_next();
}

void IMU::_init_data_sent_handler() {
  _ready_to_read = true;
}

bool IMU::_send_next() {
  if (_current < _chunks) {
    _i2c->send(&_send_buf[_current].reg_addr, _send_buf[_current].len + 1, true, &__sending_handler, this);
    return true;
  }
  return false;
}

void IMU::_sending_handler() {
  _current += 1;
  if (!_send_next())
    _call_and_clear_callback();
}

void IMU::_call_and_clear_callback() {
  if (_done_cb) {
    cb_type to_call = _done_cb;
    void *to_pass = _cb_user_data;
    _done_cb = nullptr;
    _cb_user_data = nullptr;
    to_call(to_pass);
  }
}

void IMU::read_all(cb_type done_cb, void *user_data, cb_type failed_cb) {
  if (_i2c->is_sending())
    return;

  _done_cb = done_cb;
  _cb_user_data = user_data;
  _failed_cb = failed_cb;

  _read_buf[0] = {0x43, _gyro[1 - _curr_gyro].data, 6};
  _read_buf[1] = {0x3B, _acc[1 - _curr_acc].data, 6};
  _chunks = 2;
  _current = 0;
  _read_next();
}

bool IMU::_read_next() {
  if (_current < _chunks) {
    _i2c->send(&_read_buf[_current].reg_addr, 1, false, &__reg_addr_sent_handler, this);
    return true;
  }
  return false;
}

void IMU::_reg_addr_sent_handler() {
  _i2c->read(_read_buf[_current].dest, _read_buf[_current].len, &__reading_done_handler, this);
}

void IMU::_reading_done_handler() {
  bool _reading_gyro = _read_buf[_current].dest == _gyro[1 - _curr_gyro].data;
  bool _reading_acc = _read_buf[_current].dest == _acc[1 - _curr_acc].data;
  if (_reading_gyro) {
    for (volatile int16_t &i : _gyro[1 - _curr_gyro].data16)
      i = __REV16(i);
    _curr_gyro = (1 - _curr_gyro);
  } else if (_reading_acc) {
    for (volatile int16_t &i : _acc[1 - _curr_acc].data16)
      i = __REV16(i);
    _curr_acc = (1 - _curr_acc);
  }
  _current += 1;
  if (!_read_next())
    _call_and_clear_callback();
}

Readings IMU::acc() {
  return {{
      (int16_t) clamp(_acc[_curr_acc].x - _acc_offset.x, INT16_MIN, INT16_MAX),
      (int16_t) clamp(_acc[_curr_acc].y - _acc_offset.y, INT16_MIN, INT16_MAX),
      (int16_t) clamp(_acc[_curr_acc].z - _acc_offset.z, INT16_MIN, INT16_MAX)
  }};
}

Readings IMU::gyro() {
  return {{
      (int16_t) clamp(_gyro[_curr_gyro].x - _gyro_offset.x, INT16_MIN, INT16_MAX),
      (int16_t) clamp(_gyro[_curr_gyro].y - _gyro_offset.y, INT16_MIN, INT16_MAX),
      (int16_t) clamp(_gyro[_curr_gyro].z - _gyro_offset.z, INT16_MIN, INT16_MAX)
  }};
}

Readings IMU::raw_acc() {
  return _acc[_curr_acc];
}

Readings IMU::raw_gyro() {
  return _gyro[_curr_gyro];
}

void IMU::calibrate(uint32_t probes) {
  int32_t gyro_off[3] = {0, 0, 0};
  int32_t acc_off[3] = {0, 0, 0};

  while (_i2c->is_sending());

  uint32_t calibration = probes;
  for ( ; calibration > 0; --calibration) {
    read_all();
    while (is_busy());
    Readings g = raw_gyro();
    Readings a = raw_acc();
    for (uint8_t i = 0; i < 3; ++i) {
      gyro_off[i] += g.data16[i];
      acc_off[i] += a.data16[i];
    }
    Timer::sleep(2);
  }

  for (int32_t& i : gyro_off) {
    i /= int32_t(probes);
  }

  uint8_t max_ind = 0;
  int32_t max = std::abs(acc_off[0]);
  for (uint8_t i = 1; i < 3; ++i) {
    if (acc_off[i] > max) {
      max_ind = i;
      max = std::abs(acc_off[i]);
    }
  }

  for (uint8_t i = 0; i < 3; ++i) {
    if (i != max_ind) {
      _acc_offset.data16[i] = static_cast<int16_t>(acc_off[i] / int32_t(probes));
    } else {
      _acc_offset.data16[i] = 0;
    }
  }

  _gyro_offset = {{
      static_cast<int16_t>(gyro_off[0]),
      static_cast<int16_t>(gyro_off[1]),
      static_cast<int16_t>(gyro_off[2])
  }};
}

const Readings &IMU::get_gyro_offset() const {
  return _gyro_offset;
}

const Readings &IMU::get_acc_offset() const {
  return _acc_offset;
}

void IMU::swreset(uint32_t delay_ms) {
  _i2c->swreset(delay_ms);
}

// Non class methods

void __reg_addr_sent_handler(void *_imu) {
  auto *imu = (IMU *) _imu;
  imu->_reg_addr_sent_handler();
}

void __reading_done_handler(void *_imu) {
  auto *imu = (IMU *) _imu;
  imu->_reading_done_handler();
}

void __init_data_sent_handler(void *_imu) {
  auto *imu = (IMU *) _imu;
  imu->_init_data_sent_handler();
}

void __sending_handler(void *_imu) {
  auto *imu = (IMU *) _imu;
  imu->_sending_handler();
}
