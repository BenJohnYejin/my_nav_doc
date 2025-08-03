#include "leador_os.h"
#include "rtkport.h"

extern XYRTK_Callbacks* cb;

void* xy_malloc(size_t t)
{
#if defined(WIN32)
	return malloc(t);
#else
	return cb->malloc(t);
#endif
}

void xy_free(void* p)
{
#if defined(WIN32)
	free(p);
#else
	cb->free(p);
#endif
}