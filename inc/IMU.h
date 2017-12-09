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

  // Structure used to simplify reading via I2C, written into _read_buf
  struct ReadChunk {
    uint8_t reg_addr; // register address to read from
    volatile uint8_t *dest; // destination to read into
    uint8_t len; // number of bytes to read
  };

  // Structure used to simplify writing via I2C, written into _send_buf
  struct SendChunk {
    uint8_t reg_addr; // register address to write into
    uint8_t data[8]; // data to write
    uint8_t len; // number of bytes to write
  };

  /*
   * In order to read, ReadChunks must be written into _read_buf, _chunks must be set
   * to number of chunks written and _current must be set to 0. Then, _read_next() should
   * be called.
   *
   * Similarly for sending - write SendChunks into _send_buf, set _chunks and _current
   * and call _send_next() or _send_from_buf to provied callback to be called when sending
   * of all chunks is done.
   *
   * Only sending or reading must be done at one time, not both.
   */
  ReadChunk _read_buf[8];
  SendChunk _send_buf[8];
  uint8_t _chunks = 0;
  volatile uint8_t _current = 0;

  // Double buffering, so that readings are available whole the time
  Readings _gyro[2];
  Readings _acc[2];
  // Below numbers point to latest readings in _gyro and _acc
  volatile uint8_t _curr_gyro = 0;
  volatile uint8_t _curr_acc = 0;

  // Offsets set on calibration
  Readings _gyro_offset = {{0, 0, 0}};
  Readings _acc_offset = {{0, 0, 0}};

  // Used to store user provided callbacks by read_all() or _send_from_buf
  volatile cb_type _done_cb = nullptr, _failed_cb = nullptr;
  void *volatile _cb_user_data = nullptr;

  // Starts reading of current chunk, _reading_done_handler is called after whole chunk is read
  // Returns true when next chunk was processed, false when there are no more chunks to process
  bool _read_next();

  // Starts sending of current chunk, cb will be called after whole chunk is sent
  // Calls _send_next() internally
  void _send_from_buf(cb_type cb, void *user_data);
  // Starts sending of current chunk
  // Returns true if next chunk was sent to I2C, false when there are no more chunks to process
  bool _send_next();

  // Callback for intermediate step of reading
  void _reg_addr_sent_handler();
  // Callback called after reading of chunk is done
  void _reading_done_handler();
  // Callback used for _send_from_buf when initialization is done
  void _init_data_sent_handler();
  // Callback for intermediate step of sending
  void _sending_handler();

  void _call_and_clear_callback();

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
