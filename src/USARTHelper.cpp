#include <USARTHelper.h>

#include <Context.h>

#include <cstring>
#include <cstdio>

char USARTHelper::output_buffer[64];

const char *USARTHelper::_help = R"str(Available options:
?m - motors menu
? - display this message
)str";

const char *USARTHelper::_motors = R"str(Available options:
?g<num> - get motor <num> output (0-1)
?b - go back
? - display this message
)str";

USARTHelper::USARTHelper(Context *context):
  _context(context),
  _bytes_read(0),
  _current(&USARTHelper::special)
{
  _context->uart_usb->set_rx_callback(__usart_rx_callback, this);
}

void USARTHelper::rx_callback(uint8_t byte)
{
  if (byte == '\b')
  {
    if (_bytes_read > 0)
      _bytes_read--;
  }
  else if (byte == '\n')
  {
    _buffer[_bytes_read++] = '\0';
    _bytes_read = 0;
    (this->*_current)();
  }
  else
    _buffer[_bytes_read++] = byte;
}

void USARTHelper::special()
{
  if (strcmp(_buffer, "?m") == 0)
  {
    _context->uart_usb->send(_motors);
    _current = &USARTHelper::motors;
  }
  else
    _context->uart_usb->send(_help);
}

void USARTHelper::motors()
{
  if (strcmp(_buffer, "?b") == 0)
  {
    _context->uart_usb->send(_help);
    _current = &USARTHelper::special;
  }
  else if (_buffer[0] == '?' && _buffer[1] == 'g')
  {
    switch (_buffer[2])
    {
      case '1':
      case '2':
      case '3':
      case '4':
        if (_context->motors[_buffer[2] - '0' - 1]->armed())
          sprintf(output_buffer, "%.2f (armed)\n", _context->motors[_buffer[2] - '0' - 1]->current());
        else
          sprintf(output_buffer, "%.2f (not armed)\n", _context->motors[_buffer[2] - '0' - 1]->current());
        break;
      default:
        sprintf(output_buffer, "<num> must be 1-4\n");
    }
    _context->uart_usb->send(output_buffer);
  }
  else
    _context->uart_usb->send(_motors);
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __usart_rx_callback(void *usart_helper, uint8_t byte)
{
  USARTHelper *helper = (USARTHelper*) usart_helper;
  helper->rx_callback(byte);
}
