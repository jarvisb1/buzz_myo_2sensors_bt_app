#ifndef _CONFIG_H_
#define _CONFIG_H_

#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
#define DEBUG_PRINTLN(msg) Serial.println(msg)
#else
#define DEBUG_PRINTLN(msg)
#endif

#endif