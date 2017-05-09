#ifndef __RC_H_
#define __RC_H_

#include "ch.h"
#include "hal.h"

typedef uint16_t rc_channel_t;

rc_channel_t* rc_getChannels(void);
rc_channel_t* rc_init(void);

#endif
