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

void USARTHelper::next_iter()
{
  _sending = 0;
  tx_callback();
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

void USARTHelper::tx_callback()
{
  if (_sending == 0 && _send_readings)
  {
    sprintf(output_buffer, "r %6d %6d %6d\n", _context->readings->x, _context->readings->y, _context->readings->z);
    _context->uart_usb->send(output_buffer, __usart_tx_callback, this);
  }
  else if (_sending == _send_readings && _send_ang_rates)
  {
    sprintf(output_buffer, "a %6.2f %6.2f %6.2f\n", _context->angular_rates->pitch, _context->angular_rates->roll, _context->angular_rates->yaw);
    _context->uart_usb->send(output_buffer, __usart_tx_callback, this);
  }
  else if (_sending == (_send_readings + _send_ang_rates) && _send_controls)
  {
    sprintf(output_buffer, "c %6.2f %6.2f %6.2f\n", _context->controls->pitch, _context->controls->roll, _context->controls->yaw);
    _context->uart_usb->send(output_buffer, __usart_tx_callback, this);
  }
  _sending++;
}

void USARTHelper::special()
{
  if (_buffer[0] == 'r')
    _send_readings = !_send_readings;
  else if (_buffer[0] == 'a')
    _send_ang_rates = !_send_ang_rates;
  else if (_buffer[0] == 'c')
    _send_controls = !_send_controls;
  else if (_buffer[0] == 's')
    set_PID_coeff();
  // if (strcmp(_buffer, "?m") == 0)
  // {
  //   _context->uart_usb->send(_motors);
  //   _current = &USARTHelper::motors;
  // }
  // else
  //   _context->uart_usb->send(_help);
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

void USARTHelper::set_PID_coeff()
{
  char part = '\0';
  float val = 1.f;
  if (2 != sscanf(_buffer, "s%c %f", &part, &val))
    sprintf(output_buffer, "Input must be of form 's<part> <val>'\n");
  else
  {
    switch(part)
    {
      case 'p':
      case 'P':
        _context->pid->set_coeff(SimplePID::P, val);
        sprintf(output_buffer, "Set P PID coeff to %f\n", val);
        break;
      case 'd':
      case 'D':
        _context->pid->set_coeff(SimplePID::D, val);
        sprintf(output_buffer, "Set D PID coeff to %f\n", val);
        break;
      case 'i':
      case 'I':
        _context->pid->set_coeff(SimplePID::I, val);
        sprintf(output_buffer, "Set I PID coeff to %f\n", val);
        break;
      default:
        sprintf(output_buffer, "Part must be one of 'P', 'I', 'D'\n");
    }
  }
  _context->uart_usb->send(output_buffer);
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __usart_rx_callback(void *usart_helper, uint8_t byte)
{
  USARTHelper *helper = (USARTHelper*) usart_helper;
  helper->rx_callback(byte);
}

void __usart_tx_callback(void *usart_helper)
{
  USARTHelper *helper = (USARTHelper*) usart_helper;
  helper->tx_callback();
}
