#include <util/USARTHelper.h>

#include <util/Context.h>

#include <cstring>
#include <cstdio>

const char *USARTHelper::delimit_tokens = " \r\n";
char USARTHelper::output_buffer[256];

USARTHelper::USARTHelper(USART *usart_to_use, Context *context):
  _usart(usart_to_use),
  _context(context),
  _bytes_read(0),
  _bt_passthrough(false)
{
  _usart->set_rx_callback(__usart_rx_callback, this);
}

void USARTHelper::main_menu()
{
  static const char* help = "Available commands: bt uart\n";

  char *main_cmd = strtok(_buffer, delimit_tokens);
  if (!main_cmd)
    _usart->send(help);
  else if (strcmp(main_cmd, "bt") == 0)
    bluetooth();
  else if (strcmp(main_cmd, "uart") == 0)
    uart();
  else
    _usart->send(help);
}

void USARTHelper::bluetooth()
{
  static const char* help = "Available subcommands: passthrough\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd)
    _usart->send(help);
  else if (strcmp(subcmd, "passthrough") == 0)
  {
    if (_usart == _context->uart_bt)
      _usart->send("Cannot set BT passthrough when communicating via BT\n");
    else
    {
      _usart->send("Starting passthrough\n");
      _bt_passthrough = true;
      _context->uart_bt->set_rx_callback(__usart_rx_bt_callback, this);
    }
  }
  else
    _usart->send(help);
}

void USARTHelper::uart()
{
  static const char* help = "Available subcommands: toggle\n";

  char *subcmd = strtok(nullptr, delimit_tokens);
  if (!subcmd || strlen(subcmd) == 0)
  {
    sprintf(output_buffer, "Currently communicating via %s\n", (_usart == _context->uart_bt) ? "BT" : "USB");
    _usart->send(output_buffer);
  }
  else if (strcmp(subcmd, "toggle") == 0)
  {
    USART *new_uart = (_usart == _context->uart_bt) ? _context->uart_usb : _context->uart_bt;
    sprintf(output_buffer, "Switching UART to %s\n", (new_uart == _context->uart_usb) ? "USB" : "BT");
    _usart->send(output_buffer);
    _usart->clear_rx_callback();
    _usart = new_uart;
    _usart->set_rx_callback(__usart_rx_callback, this);
  }
  else
    _usart->send(help);
}

void USARTHelper::rx_callback(uint8_t byte, USART *USART_to_forward)
{
  if (byte == '\n')
  {
    if (_bt_passthrough)
    {
      if (_bytes_read >= 3 && strncmp(_buffer, "$$$", 3) == 0)
      {
        _usart->send("Ending passthrough\n");
        _bt_passthrough = false;
        _context->uart_bt->clear_rx_callback();
        return;
      }
      else if (_bytes_read == 0 || _buffer[_bytes_read - 1] != '\r')
      {
        _buffer[_bytes_read++] = '\r';
        _bytes_read %= 64;
      }
      _buffer[_bytes_read++] = '\n';
      _bytes_read %= 64;
      _buffer[_bytes_read++] = '\0';

      if (!USART_to_forward)
        USART_to_forward = _context->uart_bt;

      USART_to_forward->send(_buffer);
    }
    else
    {
      _buffer[_bytes_read++] = '\n';
      _bytes_read %= 64;
      _buffer[_bytes_read++] = '\0';
      main_menu();
    }
    _bytes_read = 0;
  }
  else
  {
    _buffer[_bytes_read++] = byte;
    _bytes_read %= 64;
  }
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __usart_rx_callback(void *usart_helper, uint8_t byte)
{
  auto *helper = (USARTHelper*) usart_helper;
  helper->rx_callback(byte);
}

void __usart_rx_bt_callback(void *usart_helper, uint8_t byte)
{
  auto *helper = (USARTHelper*) usart_helper;
  helper->rx_callback(byte, helper->_context->uart_usb);
}
