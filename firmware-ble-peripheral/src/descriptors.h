#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#include <stdint.h>

#define NOUR_DESCRIPTORS 6

struct descriptor_def_t {
    uint8_t idx;
    const uint8_t* descriptor;
    uint32_t descriptor_length;
    uint16_t vid;
    uint16_t pid;
};

extern const descriptor_def_t descriptors[NOUR_DESCRIPTORS];

#endif
