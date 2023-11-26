#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <can.h>

typedef struct
{
    uint8_t type;
    int angle;
    int speed;
} CAN_Message_t;

void CAN_Filter_configure(uint8_t id);



#endif // !