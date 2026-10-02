#ifndef PGM_COMPAT_H
#define PGM_COMPAT_H

// On the ESP8266, string literals live in RAM unless wrapped in PSTR() and read with the *_P
// functions. The native tests have no flash/RAM split, so these map to the plain functions.
#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <stdio.h>
#include <string.h>
typedef const char* PGM_P;
#define PSTR(s) (s)
#define snprintf_P snprintf
#define strncpy_P strncpy
#endif

#endif
