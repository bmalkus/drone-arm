#include <util/USARTHelper.h>

#include <util/Context.h>

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

const char *USARTHelper::delimit_tokens = " \r\n";

char USARTHelper::output_buffer[256];

USARTHelper::USARTHelper(USART *usart_to_use) :
    _usart(usart_to_use),
    _bytes_read(0),
    _bt_passthrough(false) {
  _usart->set_rx_callback(__usart_rx_callback, this);
}

void USARTHelper::send(char c) {
  _usart->send(c);
}

void USARTHelper::send(const char *str) {
  _usart->send(str);
}

void USARTHelper::next_loop_iter() {
  static uint16_t i = 0;
  if (i++ == 200) {
    i = 0;
    if (_log_controls)
      printf("%.04f %.04f %.04f\n", Context::controls->roll, Context::controls->pitch, Context::controls->yaw);
    if (_log_inputs)
      printf("%.02f %.02f %.02f %.02f\n", Context::inputs->throttle, Context::inputs->roll, Context::inputs ->pitch, Context::inputs->yaw);
    if (_log_filtered)
      printf("%.04f %.04f %.04f\n", rad_to_deg(Context::filtered_readings->angles.roll), rad_to_deg(Context::filtered_readings->angles.pitch), rad_to_deg(Context::filtered_readings->yaw_rate));
    if (_log_gyro)
      printf("%d %d %d\n", Context::gyro->x, Context::gyro->y, Context::gyro->z);
    if (_log_acc)
      printf("%d %d %d\n", Context::acc->x, Context::acc->y, Context::acc->z);
  }
}

void USARTHelper::main_menu() {
  static const char *help = "Available commands: bt uart motor log pid imu input\n";

  char *main_cmd = strtok(_buffer, delimit_tokens);
  if (!main_cmd)
    _usart->send(help);
  else if (strcmp(main_cmd, "bt") == 0)
    bluetooth();
  else if (strcmp(main_cmd, "uart") == 0)
    uart();
  else if (strcmp(main_cmd, "motor") == 0)
    motor();
  else if (strcmp(main_cmd, "log") == 0)
    log();
  else if (strcmp(main_cmd, "pid") == 0)
    pid();
  else if (strcmp(main_cmd, "imu") == 0)
    imu();
  else if (strcmp(main_cmd, "input") == 0)
    input();
  else
    _usart->send(help);
}

void USARTHelper::bluetooth() {
  static const char *help = "Available subcommands: passthrough\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd)
    _usart->send(help);
  else if (strcmp(subcmd, "passthrough") == 0) {
    if (_usart == Context::uart_bt)
      _usart->send("Cannot set BT passthrough when communicating via BT\n");
    else {
      _usart->send("Starting passthrough\n");
      _bt_passthrough = true;
      Context::uart_bt->set_rx_callback(__usart_rx_bt_callback, this);
    }
  } else
    _usart->send(help);
}

void USARTHelper::uart() {
  static const char *help = "Available subcommands: toggle\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd || strlen(subcmd) == 0) {
    sprintf(output_buffer, "Currently communicating via %s\n", (_usart == Context::uart_bt) ? "BT" : "USB");
    _usart->send(output_buffer);
  } else if (strcmp(subcmd, "toggle") == 0) {
    USART *new_uart = (_usart == Context::uart_bt) ? Context::uart_usb : Context::uart_bt;
    sprintf(output_buffer, "Switching UART to %s\n", (new_uart == Context::uart_usb) ? "USB" : "BT");
    _usart->send(output_buffer);
    _usart->clear_rx_callback();
    _usart = new_uart;
    _usart->set_rx_callback(__usart_rx_callback, this);
  } else
    _usart->send(help);
}

void USARTHelper::motor() {
  static const char *help = "Available subcommands: enable disable toggle[arming state]\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd)
    printf(help);
  else if (strcmp(subcmd, "toggle") == 0) {
    Context::stick_inputs->_should_be_armed = !Context::stick_inputs->_should_be_armed;
  } else {
    bool enable = (strcmp(subcmd, "enable") == 0);
    bool disable = (strcmp(subcmd, "disable") == 0);
    if (enable || disable) {
      char *num = strtok(nullptr, delimit_tokens);
      if (!num) {
        printf("Syntax: motor %s <1-4>\n", enable ? "enable" : "disable");
      } else {
        long n = strtol(num, nullptr, 10);
        if (n < 1 || n > 4) {
          printf("Syntax: motor %s <1-4>\n", enable ? "enable" : "disable");
        } else if (disable) {
          if (Context::motors[n - 1]->is_disabled()) {
            printf("Already disabled\n");
          } else {
            Context::motors[n - 1]->disable();
            printf("Disabled motor %ld\n", n);
          }
        } else {
          if (!Context::motors[n - 1]->is_disabled()) {
            printf("Already enabled\n");
          } else {
            Context::motors[n - 1]->enable();
            printf("Enabled motor %ld\n", n);
          }
        }
      }
    } else
      printf(help);
  }
}

void USARTHelper::log() {
  static const char *help = "Available subcommands: inputs filtered controls gyro acc\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd)
    printf(help);
  else if (strcmp(subcmd, "inputs") == 0)
    _log_inputs = !_log_inputs;
  else if (strcmp(subcmd, "filtered") == 0)
    _log_filtered = !_log_filtered;
  else if (strcmp(subcmd, "controls") == 0)
    _log_controls = !_log_controls;
  else if (strcmp(subcmd, "gyro") == 0)
    _log_gyro = !_log_gyro;
  else if (strcmp(subcmd, "acc") == 0)
    _log_acc = !_log_acc;
  else
    _usart->send(help);
}

void USARTHelper::pid() {
  static const char *help = "Available subcommands: set get\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd) {
    printf(help);
  } else if (strcmp(subcmd, "get") == 0) {
    printf_pid_coeffs();
  } else if (strcmp(subcmd, "set") == 0) {
    char *part = strtok(nullptr, delimit_tokens);
    if (!part) {
      printf("Syntax: pid set <part> <value>");
      return;
    }
    char *val = strtok(nullptr, delimit_tokens);
    if (!val) {
      printf("Syntax: pid set <part> <value>");
      return;
    }
    float f = strtof(val, nullptr);
    if (f < 0.f) {
      printf("Value must be float >= 0");
      return;
    }
    if (strcmp(part, "p") == 0) {
      Context::pid->set_coeff(AnglePID::P, f);
      printf_pid_coeffs();
    } else if (strcmp(part, "i") == 0) {
      Context::pid->set_coeff(AnglePID::I, f);
      printf_pid_coeffs();
    } else if (strcmp(part, "d") == 0) {
      Context::pid->set_coeff(AnglePID::D, f);
      printf_pid_coeffs();
    } else {
      printf("Syntax: pid set <part> <value>");
    }
  } else {
    _usart->send(help);
  }
}

void USARTHelper::printf_pid_coeffs() {
  printf("    P    |    I    |    D\n");
  printf(
      " %.05f | %.05f | %.05f\n", Context::pid->get_coeff(AnglePID::P), Context::pid
          ->get_coeff(AnglePID::I), Context::pid->get_coeff(AnglePID::D));
}

void USARTHelper::imu() {
  static const char *help = "Available subcommands: calib\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd) {
    printf(help);
  } else if (strcmp(subcmd, "calib") == 0) {
    Context::stick_inputs->_should_calibrate = true;
  } else {
    printf(help);
  }
}

void USARTHelper::input() {
  static const char *help = "Available subcommands: ignore[disabled TX]\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd) {
    printf(help);
  } else if (strcmp(subcmd, "ignore") == 0) {
    Context::stick_inputs->_ignore_disabled_tx = !Context::stick_inputs->_ignore_disabled_tx;
    if (Context::stick_inputs->_ignore_disabled_tx)
      printf("WARN: ignoring disabled TX\n");
    else
      printf("Not ignoring disabled TX anymore\n");
  } else {
    printf(help);
  }
}

void USARTHelper::rx_callback(uint8_t byte, USART *USART_to_forward) {
  if (byte == '\n') {
    if (_bt_passthrough) {
      if (_bytes_read >= 3 && strncmp(_buffer, "$$$", 3) == 0) {
        _usart->send("Ending passthrough\n");
        _bt_passthrough = false;
        Context::uart_bt->clear_rx_callback();
        return;
      } else if (_bytes_read == 0 || _buffer[_bytes_read - 1] != '\r') {
        _buffer[_bytes_read++] = '\r';
        _bytes_read %= 64;
      }
      _buffer[_bytes_read++] = '\n';
      _bytes_read %= 64;
      _buffer[_bytes_read++] = '\0';

      if (!USART_to_forward)
        USART_to_forward = Context::uart_bt;

      USART_to_forward->send(_buffer);
    } else {
      _buffer[_bytes_read++] = '\n';
      _bytes_read %= 64;
      _buffer[_bytes_read++] = '\0';
      main_menu();
    }
    _bytes_read = 0;
  } else {
    _buffer[_bytes_read++] = byte;
    _bytes_read %= 64;
  }
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __usart_rx_callback(void *usart_helper, uint8_t byte) {
  auto *helper = (USARTHelper *) usart_helper;
  helper->rx_callback(byte);
}

void __usart_rx_bt_callback(void *usart_helper, uint8_t byte) {
  auto *helper = (USARTHelper *) usart_helper;
  helper->rx_callback(byte, Context::uart_usb);
}

int __io_putchar(int c) {
  Context::usart_helper->send((uint8_t) c);
  return c;
}
