#ifndef _IMM_VECTOR_H_
#define _IMM_VECTOR_H_

#include "imm_types.h"

IMM_BEGIN_DECLS

	typedef struct vector_tag {
		uint8_t  *data;
		uint32_t len;
} vector_t;

typedef struct ptr_vector_tag {
	void     **pdata;
	uint32_t len;
} ptr_vector_t;

typedef int32_t(* comp_func_t)(const void *a,  const void *b);

#define vector_append_element(a,v)	     vector_append_elements (a, &(v), 1)
#define vector_at(a,t,i)                 (((t*) (void *) (a)->data) [(i)])
#define vector_size(a)                   ((a)->len)

#define ptr_vector_at(a,index_)          ((a)->pdata)[index_]
#define ptr_vector_size(a)               ((a)->len)

// Create new vector
vector_t* vector_new (uint32_t element_size);

// 
void vector_clear(vector_t *vector);

// Free the memory allocated for the vector	
void vector_free (vector_t *vector);

// Get the memory used for each element	
uint32_t vector_get_element_size  (vector_t *vector);

// Append elements to the end of the vector	
vector_t* vector_append_elements (vector_t *vector, const void *data, uint32_t len);

// Resize the vector with new length	
vector_t* vector_resize (vector_t *vector, uint32_t length);

// Remove the i-th element from the vector
// Note (Tao Li): Memory allocated for the element will not be freed. Please use the vector_shrink_to_fit() function to 
// free the unused memory.
vector_t* vector_remove_element (vector_t *vector, uint32_t i);

// Remove N elements starting from the index_  in the vector  
vector_t* vector_remove_elements (vector_t *vector, uint32_t index_, uint32_t N);

// Sort the vector with the used defined compare function	
void vector_sort (vector_t *fvector, comp_func_t compare_func);

// Create a new pointer vector
ptr_vector_t* ptr_vector_new (void);

// Free the memory for the pointer vector	
void** ptr_vector_free	(ptr_vector_t *vector, bool_t free_segment);

// Clear 
void ptr_vector_clear (ptr_vector_t *vector, bool_t free_segment);

// Resize the pointer vector with the new length
void ptr_vector_resize (ptr_vector_t *vector, uint32_t length);

// Append the pointer vector with the new pointer
void ptr_vector_append_element(ptr_vector_t *vector, void *data);

// Remove the i-th element 	
void* ptr_vector_remove_element (ptr_vector_t *vector, uint32_t  index_);

// Remove the given number of elements starting at the given index
ptr_vector_t* ptr_vector_remove_elements (ptr_vector_t *vector, uint32_t  index_, uint32_t N);

// Remove the element which points to data 	
bool_t ptr_vector_remove (ptr_vector_t *vector, void *data);

// User defined sort function for pointer vector.	
void ptr_vector_sort (ptr_vector_t *vector, comp_func_t compare_func );

bool_t ptr_vector_empty(ptr_vector_t *vector);



IMM_END_DECLS


#endif