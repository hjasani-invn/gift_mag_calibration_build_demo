/*
 *	Copyright (c) 2008 ,
 *		Cloud Wu . All rights reserved.
 *
 *		http://www.codingnow.com
 *
 *	Use, modification and distribution are subject to the "New BSD License"
 *	as listed at <url: http://www.opensource.org/licenses/bsd-license.php >.
 */

#ifndef MANUAL_GARBAGE_COLLECTOR_H
#define MANUAL_GARBAGE_COLLECTOR_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(WIN32) && !defined(EXTERN_SYMBOL)
#define EXTERN_SYMBOL __declspec(dllexport)
#elif !defined(EXTERN_SYMBOL)
#define EXTERN_SYMBOL extern
#endif

EXTERN_SYMBOL void gc_exit_and_destroy_everything();

EXTERN_SYMBOL void* gc_malloc(intptr_t sz);
EXTERN_SYMBOL void* gc_realloc(void *p, intptr_t sz);
EXTERN_SYMBOL void gc_free(void *p);

#endif
