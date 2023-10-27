#ifndef _IMM_VECTOR_H_
#define _IMM_VECTOR_H_

#include "imm_types.h"

IMM_BEGIN_DECLS


typedef struct vector_tag {
    uint8_t  *data;
    int      len;
} vector_t;


typedef struct ptr_vector_tag {
    void     **pdata;
    int      len;
} ptr_vector_t;


typedef int (*comp_func_t)(const void *a, const void *b);


#if defined(_DEBUG)
    #include <assert.h>
    #define vector_append_element(a,v)	            (assert(vector_get_element_size(a) == sizeof(v)), vector_append_elements(a, &(v), 1))
    #define vector_append_tmp_element(a, T, v)	    do { assert(vector_get_element_size(a) == sizeof(v)); T tmp = (v); vector_append_elements(a, &tmp, 1); } while (0)

    #define vector_at(a,t,i)                        (((t*) (void *)(assert((a)->len >= 0 && (i) < (a)->len), (a)->data)) [(i)])
    #define ptr_vector_at(a,index_)                 ((assert((a)->len >= 0 && (index_) < (a)->len), (a)->pdata)[(index_)])

    #define vector_reverse_at(a,t,i)                (((t*) (void *)(assert((a)->len >= 0 && (i) < (a)->len), (a)->data)) [(a)->len - (i) - 1])
    #define ptr_vector_reverse_at(a,index_)         ((assert((a)->len >= 0 && (index_) < (a)->len), (a)->pdata)[(a)->len - (index_) - 1])
#else
    #define vector_append_element(a,v)	            (vector_append_elements (a, &(v), 1))
    #define vector_append_tmp_element(a, T, v)	    do { T tmp = (v); vector_append_elements(a, &tmp, 1); } while (0)

    #define vector_at(a,t,i)                        (((t*) (void *)(a)->data) [(i)])
    #define ptr_vector_at(a,index_)                 (((a)->pdata)[(index_)])

    #define vector_reverse_at(a,t,i)                (((t*) (void *)(a)->data) [(a)->len - (i) - 1])
    #define ptr_vector_reverse_at(a,index_)         (((a)->pdata)[(a)->len - (index_) - 1])
#endif


#define vector_size(a)                          ((a)->len)
#define ptr_vector_size(a)                      ((a)->len)

#define vector_is_non_empty(v)			        (v != NULL && v->data != NULL && (v->len) > 0)
#define ptr_vector_is_non_empty(v)			    (v != NULL && v->pdata != NULL && (v->len) > 0)


// Create new vector
EXTERN_SYMBOL vector_t* vector_new(size_t element_size);

// Reset the length of the vector
EXTERN_SYMBOL void vector_clear(vector_t *vector);

// Free the memory allocated for the vector
EXTERN_SYMBOL void vector_free(vector_t *vector);

// Get the memory used for each element
EXTERN_SYMBOL size_t vector_get_element_size(vector_t *vector);

// Append elements to the end of the vector
EXTERN_SYMBOL vector_t* vector_append_elements(vector_t *vector, const void *data, int len);

// Resize the vector with new length
EXTERN_SYMBOL vector_t* vector_resize(vector_t *vector, int length);

// Remove the i-th element from the vector
// Note (Tao Li): Memory allocated for the element will not be freed. Please use the vector_shrink_to_fit() function to
// free the unused memory.
EXTERN_SYMBOL vector_t* vector_remove_element(vector_t *vector, int i);

// Remove N elements starting from the index_  in the vector
EXTERN_SYMBOL vector_t* vector_remove_elements(vector_t *vector, int index_, int N);

// Sort the vector with the used defined compare function
EXTERN_SYMBOL void vector_sort(vector_t *fvector, comp_func_t compare_func);

EXTERN_SYMBOL void vector_clone(vector_t *dest, vector_t *src);


// Create a new pointer vector
EXTERN_SYMBOL ptr_vector_t* ptr_vector_new(void);

// Free the memory for the pointer vector
EXTERN_SYMBOL void** ptr_vector_free(ptr_vector_t *vector, bool free_segment);

// Clear
EXTERN_SYMBOL void ptr_vector_clear(ptr_vector_t *vector, bool free_segment);

// Resize the pointer vector with the new length
EXTERN_SYMBOL void ptr_vector_resize(ptr_vector_t *vector, int length);

// Append the pointer vector with the new pointer
EXTERN_SYMBOL void ptr_vector_append_element(ptr_vector_t *vector, void *data);
EXTERN_SYMBOL void ptr_vector_append_elements(ptr_vector_t *vector, void **data, int len);

// Remove the i-th element
EXTERN_SYMBOL void* ptr_vector_remove_element(ptr_vector_t *vector, int  index_);

// Remove the given number of elements starting at the given index
EXTERN_SYMBOL ptr_vector_t* ptr_vector_remove_elements(ptr_vector_t *vector, int  index_, int N);

// Remove the element which points to data
EXTERN_SYMBOL bool ptr_vector_remove(ptr_vector_t *vector, void *data);

// User defined sort function for pointer vector.
EXTERN_SYMBOL void ptr_vector_sort(ptr_vector_t *vector, comp_func_t compare_func);


EXTERN_SYMBOL void ptr_vector_clone(ptr_vector_t *dest, ptr_vector_t *src);

EXTERN_SYMBOL bool ptr_vector_empty(ptr_vector_t *vector);

EXTERN_SYMBOL bool ptr_vector_is_in(ptr_vector_t *vector, void* data);

EXTERN_SYMBOL void ptr_vector_reverse_in_place(ptr_vector_t *v);

IMM_END_DECLS


#endif