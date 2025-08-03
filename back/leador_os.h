#ifndef _LEADOROS_H
#define _LEADOROS_H
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>

/* defined your system platform  -----------------------------------------------------------------*/
//#define WIN32
/* -----------------------------------------------------------------------------------------------*/

#ifndef FILE
#define FILE char
#endif

#if defined(WIN32)
#pragma warning(disable:4996)
#define _POSIX_C_SOURCE 199506
#include <winsock2.h>
#include <windows.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <winsock2.h>
#include <windows.h>
#endif

void* xy_malloc(size_t t);
void xy_free(void* p);

#endif
