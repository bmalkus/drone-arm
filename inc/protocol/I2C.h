#ifndef I2C_H
#define I2C_H

#include <cstdint>

#include <stm32f4xx.h>

#include <util/HandlerHelper.h>
#include <util/misc.h>

/**
 * @brief Class responsible for managing I2C interface
 *
 * Note: Appropriate IO pins must be configured separately in order to I2C work properly,
 * as such configuration lays beyond this class responsibility
 */
class I2C {
public:
  /**
   * @brief Constructor
   *
   * @param I2C pointer to CMSIS I2C struct that will be used
   */
  I2C(I2C_TypeDef *I2C);

  /**
   * @brief Initializes I2C peripheral
   *
   * @param APB_freq_MHz
   * @param fs
   * @param duty
   */
  void init(uint32_t APB_freq_MHz, bool fs, uint8_t duty);

  /**
   * @brief Sets address of slave device to communicate with
   *
   * @param addr Address of slave device to communicate with
   */
  void set_addr(uint8_t addr);

  /**
   * @brief Sends single byte of data to slave device - asynchronous
   *
   * @param byte Byte to send
   * @param do_stop_cond Indicates if stop condition should be performed after sending
   * @param cb Callback to call after sending finishes
   * @param user_data User data passed to callback
   */
  bool send(uint8_t byte, bool do_stop_cond = true, cb_type cb = nullptr, void *user_data = nullptr);
  /**
   * @brief Send stream of bytes to slave device - asynchronous
   *
   * @param bytes Bytes to send
   * @param len Number of bytes to send
   * @param do_stop_cond Indicates if stop condition should be performed after sending
   * @param cb Callback to call after sending finishes
   * @param user_data User data passed to callback
   */
  bool send(const uint8_t *bytes,
            uint8_t len,
            bool do_stop_cond = true,
            cb_type cb = nullptr,
            void *user_data = nullptr);

  /**
   * @brief Read one or more bytes from slave device - asynchronous
   *
   * @param buf Destination buffer
   * @param len Number of bytes to read
   * @param cb Callback to call after reading finishes
   * @param user_data User data passed to callback
   */
  bool read(volatile uint8_t *buf, int len, cb_type cb = nullptr, void *user_data = nullptr);

  /**
   * @brief Aborts current operation and performs stop condition
   */
  void abort_sending();

  /**
   * @brief Indicates if I2C is currently sending
   *
   * @return true if I2C is currently sending, false otherwise
   */
  bool is_sending() { return READ_BIT(_I2C->CR2, I2C_CR2_ITBUFEN); }

private:
  enum RW {
    WRITE = 0,
    READ = 1
  };

  I2C_TypeDef *_I2C;
  uint8_t _slave_addr;
  volatile RW _current_rw;

  const uint8_t *volatile _to_send;
  volatile uint8_t _byte_to_send;
  volatile uint8_t _len;
  volatile bool _do_stop_cond;
  volatile cb_type _done_cb;
  void *volatile _user_data;

  volatile uint8_t *volatile _read_buf;

  void start_cond(RW rw);
  void stop_cond();

  void handle_event(HandlerHelper::InterruptType itype);

  friend void __handle_i2c_event(HandlerHelper::InterruptType, void *);
};

void __handle_i2c_event(HandlerHelper::InterruptType, void *);

#endif /* I2C_H */
