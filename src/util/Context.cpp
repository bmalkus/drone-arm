#include <util/Context.h>

USART *Context::uart_usb = nullptr;
USART *Context::uart_bt = nullptr;
USARTHelper *Context::usart_helper = nullptr;
Motor *Context::motors[4] = {nullptr, nullptr, nullptr, nullptr};
Readings *Context::readings = nullptr;
EulerianAngles *Context::eulerian_angles = nullptr;
Controls *Context::controls = nullptr;
Sticks *Context::inputs = nullptr;
AnglePID *Context::pid = nullptr;
