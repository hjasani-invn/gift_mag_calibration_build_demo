#include "imm_memory.h"
#include "imm_gc.h"

#include <stdlib.h>

#ifdef _DEBUG
#include <stdio.h>
#endif // DEBUG      


void* 
mem_allocate (size_t mem_size)
{
    void *mem;
    mem = malloc(mem_size);

    if (mem) {
        return mem;
    }
    else {
        #ifdef _DEBUG
            printf("Error allocate memory.\n");
        #endif // DEBUG      
        exit(-1);
    }

}


void* 
mem_reallocate(void *mem, size_t mem_size)
{
    mem = realloc(mem, mem_size);
    
    if (mem) {
        return mem;
    }
    else {
        #ifdef _DEBUG
            printf("Error reallocate memory.\n");
        #endif // DEBUG      
        exit(-1);
    }
}


void 
mem_free (void* mem)
{
    return_if_fail (mem);
    free(mem);
}


void  
mem_clean_and_exit(void)
{
    gc_exit_and_destroy_everything();
}