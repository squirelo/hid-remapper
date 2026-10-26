#ifndef JOYCON_TEST_BTSTACK_H
#define JOYCON_TEST_BTSTACK_H

#include <stdint.h>

#define ERROR_CODE_SUCCESS 0

#ifdef __cplusplus
extern "C" {
#endif
uint8_t hid_host_send_report(uint16_t cid, uint16_t report_id, const uint8_t* report, uint8_t len);
#ifdef __cplusplus
}
#endif

#endif
