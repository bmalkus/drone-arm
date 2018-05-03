#include <util/Context.h>

USART *Context::uart_usb = nullptr;
USART *Context::uart_bt = nullptr;
USARTHelper *Context::usart_helper = nullptr;
Motors *Context::motors[4] = {nullptr, nullptr, nullptr, nullptr};
Readings *Context::readings = nullptr;
AngularRates *Context::angular_rates = nullptr;
Controls *Context::controls = nullptr;
RatePID *Context::pid = nullptr;
