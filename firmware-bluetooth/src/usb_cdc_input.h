#ifndef USB_CDC_INPUT_H_
#define USB_CDC_INPUT_H_

#include <stdint.h>

#define CDC_VIRTUAL_INTERFACE 0x7e00

void usb_cdc_input_init(void);
void usb_cdc_input_poll(void);

#endif
