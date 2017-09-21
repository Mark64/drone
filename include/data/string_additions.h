// adds important string manipulation functions missing from the standard libraries
// by Mark Hill
#ifndef __string_additions_h
#define __string_additions_h

#include<stddef.h>

// from libbsd
// see 'man strlcpy'
size_t strlcpy(char *dst, const char *src, size_t size);

#endif
