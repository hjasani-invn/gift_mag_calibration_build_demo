#ifndef _IMM_MEMORY_H_
#define _IMM_MEMORY_H_

#include "imm_types.h"

IMM_BEGIN_DECLS

// Allocate the memory with the size of the structure "type", return the pointer casting to "type"
#define mem_slice_new(type)      ((type*) mem_allocate (sizeof (type)))

// Allocate the memory with the size of mem_size
void* mem_allocate (uint32_t mem_size);

// Reallocate the memory with the size of the mem_size
void* mem_reallocate (void *memory, uint32_t mem_size);

// Free the allocated memeory
void  mem_free (void *memeory);

IMM_END_DECLS

#endif