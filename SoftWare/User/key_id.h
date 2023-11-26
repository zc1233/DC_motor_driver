#ifndef _INCLUDE_
#define _INCLUDE_

#include <stdint.h>

#define ID_ADDR 0x0801FC00

extern uint8_t ID;

void IDLEDAndIDSet(void const * argument);

void ID_init(void);
void WriteID(uint8_t id);

#endif

