#include <stdlib.h>

#include "imm_memory.h"

void* 
mem_allocate (uint32_t mem_size)
{
	void *mem;
	mem = malloc(mem_size);

	if (mem)
		return mem;
	else
		printf("Error allocate memory.\n");

	return NULL;
}

void* 
mem_reallocate (void *mem, uint32_t mem_size)
{
	mem = realloc(mem, mem_size);
	if (mem)
		return mem;
	else
		printf("Error reallocate memory.\n");

	return NULL;	
}

void 
mem_free (void* mem)
{
	return_if_fail (mem);
	free(mem);
}