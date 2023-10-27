#ifndef _IMM_MEMORY_H_
#define _IMM_MEMORY_H_

#include "imm_types.h"

IMM_BEGIN_DECLS

// Allocate the memory with the size of the structure "type", return the pointer casting to "type"
#define mem_slice_new(type)      ((type*) mem_allocate (sizeof (type)))

// Allocate the memory with the size of mem_size
EXTERN_SYMBOL void* mem_allocate(size_t mem_size);

// Reallocate the memory with the size of the mem_size
EXTERN_SYMBOL void* mem_reallocate(void *memory, size_t mem_size);

// Free the allocated memeory
EXTERN_SYMBOL void  mem_free(void *memeory);


EXTERN_SYMBOL void  mem_clean_and_exit(void);

IMM_END_DECLS

#endif