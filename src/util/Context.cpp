#include <util/Context.h>

USART *Context::uart_usb = nullptr;

USART *Context::uart_bt = nullptr;

USARTHelper *Context::usart_helper = nullptr;

Motor *Context::motors[4] = {nullptr, nullptr, nullptr, nullptr};

Readings *Context::readings = nullptr;

FilteredReadings *Context::filtered_readings = nullptr;

Controls *Context::controls = nullptr;

Sticks *Context::inputs = nullptr;

StickInputs *Context::stick_inputs = nullptr;

AnglePID *Context::pid = nullptr;

Readings *Context::gyro = nullptr;

Readings *Context::acc = nullptr;
