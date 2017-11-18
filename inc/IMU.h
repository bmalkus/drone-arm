#ifndef IMU_H
#define IMU_H

#include <I2C.h>

/**
 * @brief Union used to store readings from sensors
 */
union Readings {
  struct {
    volatile int16_t x, y, z;
  };
  volatile uint8_t data[6];
  volatile int16_t data16[3];
};

/**
 * @brief Class responsible for managing IMU via i2c interface
 *
 * Note: Appropriate IO pins must be configured separately in order to I2C work properly,
 * as such configuration lays beyond this class responsibility
 */
class IMU {
public:
  /**
   * @brief Constructor
   *
   * @param i2c pointer to I2C object that will be used to communicate with IMU
   */
  IMU(I2C *i2c);

  /**
   * @brief Initializes both I2C peripheral and IMU itself - asynchronous (see ready_to_read())
   */
  void init();

  /**
   * @brief Calibrates IMU (gyro and acc) - synchronous
   *
   * Calibration must be done when IMU is standing still, as it may result with
   * false further reading otherwise
   *
   * @param probes Number of samples to use to calibrate
   */
  void calibrate(uint32_t probes);

  /**
   * @brief Reads both gyro and acc - asynchronous
   *
   * @param done_cb callback called after reading is done with #user_data as parameter
   * @param user_data parameter to callback called on completion or failure
   * @param failed_cb callback called when error is encountered when reading with #user_data as parameter
   *                  (currently unused)
   */
  void read_all(cb_type done_cb=nullptr, void *user_data=nullptr, cb_type failed_cb=nullptr);

  /**
   * @brief Get most recent acc readings offseted by calibration result (see calibrate())
   *
   * @return #Readings from acc
   */
  Readings acc();
  /**
   * @brief Get most recent gyro readings offseted by calibration result (see calibrate())
   *
   * @return #Readings from gyro
   */
  Readings gyro();

  /**
   * @brief Get most recent raw acc readings (NOT offseted by calibration result)
   *
   * @return #Readings from acc
   */
  Readings raw_acc();
  /**
   * @brief Get most recent raw gyro readings (NOT offseted by calibration result)
   *
   * @return #Readings from gyro
   */
  Readings raw_gyro();

  /**
   * @brief Indicates if initialization finished (see init())
   *
   * @return true if initialization finished, false otherwise
   */
  bool ready_to_read() { return _ready_to_read; }

  /**
   * @brief Indicates if IMU is busy sending/receiving data
   *
   * @return true if sending/receiving, false otherwise
   */
  bool is_busy() { return _current != _chunks; }

private:
  I2C *_i2c;

  volatile bool _ready_to_read = false;

  struct ReadChunk {
    uint8_t reg_addr;
    volatile uint8_t *dest;
    uint8_t len;
  };

  struct SendChunk {
    uint8_t reg_addr;
    uint8_t data[8];
    uint8_t len;
  };

  ReadChunk _read_buf[8];
  SendChunk _send_buf[8];
  uint8_t _chunks = 0;
  volatile uint8_t _current = 0;

  Readings *volatile _read_target_buf = nullptr;

  Readings _gyro[2];
  Readings _acc[2];
  volatile uint8_t _curr_gyro = 0;
  volatile uint8_t _curr_acc = 0;

  Readings _gyro_offset = {{0, 0, 0}};
  Readings _acc_offset = {{0, 0, 0}};

  volatile cb_type _done_cb = nullptr, _failed_cb = nullptr;
  void *volatile _cb_user_data = nullptr;

  bool _read_next();

  void _send_from_buf(cb_type cb, void *user_data);
  bool _send_next();

  void _reg_addr_sent_handler();
  void _reading_done_handler();
  void _init_data_sent_handler();
  void _sending_handler();

  friend void __reg_addr_sent_handler(void *);
  friend void __reading_done_handler(void *);
  friend void __init_data_sent_handler(void *);
  friend void __sending_handler(void *);
};

void __reg_addr_sent_handler(void *);
void __reading_done_handler(void *);
void __init_data_sent_handler(void *);
void __sending_handler(void *);

#endif /* IMU_H */
