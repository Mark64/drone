// string functions implementation
// by Mark Hill

#include<stddef.h>

#include<string_additions.h>

size_t strlcpy(char *dst, const char *src, size_t size) {
	if (size <= 0)
		return 0;

	register char *d = dst;
	register const char *s = src;

	while ((*d++ = *s++) && s - src < size)
		;
	*--d = '\0';
	while (*(s - 1))
		s++;
	return s - src - 1;
}
