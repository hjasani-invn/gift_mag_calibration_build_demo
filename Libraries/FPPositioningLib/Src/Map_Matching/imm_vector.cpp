#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "imm_vector.h"
#include "imm_memory.h"


#define MIN_ARRAY_SIZE  32


#define vector_elt_len(vector,i)  ((vector)->elt_size * (i))
#define vector_elt_pos(vector,i)  ((vector)->data + vector_elt_len((vector),(i)))
#define vector_elt_zero(vector, pos, len)                               \
	(memset (vector_elt_pos ((vector), pos), 0,  vector_elt_len ((vector), len)))


#define ptr_vector_elt_len(i)  (sizeof(intptr_t) * (i))

typedef struct {
    uint8_t	    *data;
    int         len;
    size_t      alloc;
    size_t      elt_size;
    char        identifier[8];
} real_vector_t;


typedef struct {
    void        **pdata;
    int         len;
    size_t      alloc;
} real_ptr_vector_t;


static size_t nearest_pow(size_t num);
static void vector_maybe_expand (real_vector_t *vector, int len);
static void ptr_vector_maybe_expand (real_ptr_vector_t *vector, int len);


static size_t
nearest_pow (size_t num)
{
    size_t n = 1;

    while (n < num && n > 0)
        n <<= 1;

    return n ? n : num;
}


static void
vector_maybe_expand (real_vector_t *vector, int len)
{
    size_t want_alloc = vector_elt_len (vector, vector->len + len);

    if (want_alloc > vector->alloc) {
        want_alloc = nearest_pow (want_alloc);
        want_alloc = MAX (want_alloc, MIN_ARRAY_SIZE);

        vector->data = (uint8_t *)mem_reallocate(vector->data, want_alloc);

        memset (vector->data + vector->alloc, 0, want_alloc - vector->alloc);

        vector->alloc = want_alloc;
    }
}


void
vector_clone(vector_t *dest, vector_t *src)
{
	if (!(dest && src && vector_get_element_size(dest) == vector_get_element_size(src)))
        return;
        
    vector_clear(dest);

	vector_resize(dest, src->len);

	memmove(dest->data, src->data, src->len * vector_get_element_size(src));
}


vector_t*
vector_shrink_to_fit (vector_t *fvector)
{
    real_vector_t *vector = (real_vector_t *) fvector;
    size_t true_ussage;

    return_val_if_fail (vector, NULL);

    true_ussage = vector_elt_len (vector, vector->len);

    if (true_ussage < vector->alloc) {
        vector->alloc = MAX(true_ussage, MIN_ARRAY_SIZE);
        vector->data = (uint8_t *)mem_reallocate(vector->data, vector->alloc);

    }

    return fvector;
}


vector_t*
vector_new (size_t elt_size)
{
    real_vector_t *vector;

    return_val_if_fail (elt_size > 0, (vector_t*)NULL);

    vector = mem_slice_new (real_vector_t);

    vector->data            = NULL;
    vector->len             = 0;
    vector->alloc           = 0;
    vector->elt_size        = elt_size;
	strcpy(vector->identifier, "VECTOR");

    return (vector_t*) vector;
}


size_t
vector_get_element_size (vector_t *vector)
{
    real_vector_t *rvector = (real_vector_t *) vector;

    return_val_if_fail (vector, 0);

    return rvector->elt_size;
}


vector_t*
vector_append_elements (vector_t *fvector, const void *data, int len)
{
    real_vector_t *vector = (real_vector_t *) fvector;

    return_val_if_fail (vector, NULL);

    vector_maybe_expand (vector, len);

    memcpy (vector_elt_pos (vector, vector->len), data,
            vector_elt_len (vector, len));

    vector->len += len;

    return fvector;
}


vector_t*
vector_resize (vector_t *fvector, int  length)
{
    real_vector_t *vector = (real_vector_t *) fvector;

    return_val_if_fail (vector, NULL);

    if (length > vector->len) {
        vector_maybe_expand (vector, length - vector->len);
        vector_elt_zero (vector, vector->len, length - vector->len);
    }

    vector->len = length;

    return fvector;
}


vector_t*
vector_remove_element (vector_t *fvector, int index_)
{
    real_vector_t* vector = (real_vector_t*) fvector;

    return_val_if_fail (vector, NULL);

    return_val_if_fail (index_ < vector->len, NULL);

    if (index_ != vector->len - 1) {
        memmove(vector_elt_pos(vector, index_),
            vector_elt_pos(vector, index_ + 1),
            vector_elt_len(vector, vector->len - index_ - 1));
    }

    vector->len -= 1;

    return fvector;
}


vector_t*
vector_remove_elements (vector_t *fvector, int index_, int length)
{
    real_vector_t *vector = (real_vector_t*) fvector;

    return_val_if_fail (vector, NULL);
    return_val_if_fail (index_ < vector->len, NULL);
    return_val_if_fail (index_ + length <= vector->len, NULL);

    if (index_ + length != vector->len) {
        memmove(vector_elt_pos(vector, index_),
            vector_elt_pos(vector, index_ + length),
            (vector->len - (index_ + length)) * vector->elt_size);
    }

    vector->len -= length;

    return fvector;
}


void
vector_clear (vector_t *vector)
{
    vector->len = 0;
}


void
vector_free (vector_t *vector)
{
    real_vector_t *fvector = (real_vector_t*) vector;

    return_if_fail (fvector);

    mem_free (fvector->data);

    mem_free (fvector);
}


void
vector_sort (vector_t *vector, comp_func_t compare_func)
{
    real_vector_t *fvector = (real_vector_t*) vector;

    return_if_fail (fvector != NULL);

    qsort (fvector->data, fvector->len, fvector->elt_size, compare_func);
}


// ---------------------------------ptr_vector---------------------------------------------
static void
ptr_vector_maybe_expand (real_ptr_vector_t *vector, int len)
{
    if ((size_t)(vector->len + len) > vector->alloc) {
        size_t old_alloc = vector->alloc;
        vector->alloc = nearest_pow (vector->len + len);
        vector->alloc = MAX (vector->alloc, MIN_ARRAY_SIZE /sizeof (void *));
        vector->pdata = (void **)mem_reallocate(vector->pdata, sizeof(intptr_t) * vector->alloc);

        for ( ; old_alloc < vector->alloc; old_alloc++)
            vector->pdata [old_alloc] = NULL;
    }

}


ptr_vector_t*
ptr_vector_new (void)
{
    real_ptr_vector_t *vector;

    vector = mem_slice_new (real_ptr_vector_t);

    vector->pdata = NULL;
    vector->len = 0;
    vector->alloc = 0;

    return (ptr_vector_t*) vector;
}


void
ptr_vector_clear(ptr_vector_t *vector, bool free_segment)
{
    int i;
    real_vector_t* real_vector;

    return_if_fail (vector);

    for (i = 0; i < vector->len; i++) {
        if (free_segment) {
            real_vector = (real_vector_t *) ptr_vector_at (vector, i);
            if (real_vector != NULL) {
                if ( strcmp(real_vector->identifier, "VECTOR") == 0) {
                    vector_free ((vector_t *) ptr_vector_at (vector, i));
                } else {
                    mem_free (ptr_vector_at (vector, i));
                }
            }
        }

        ptr_vector_at(vector, i) = NULL;
    }

    vector->len = 0;
}


void**
ptr_vector_free	(ptr_vector_t *vector, bool free_segment)
{
    void **segment;

    return_val_if_fail (vector, NULL);

    // Future work
    if (free_segment) {
        //ptr_vector_clear (vector, true);
        mem_free (vector->pdata);
        segment = NULL;
    } else
        segment = vector->pdata;

    mem_free (vector);

    return segment;
}


void
ptr_vector_resize (ptr_vector_t *vector, int length)
{
    real_ptr_vector_t *rvector = (real_ptr_vector_t *)vector;

    return_if_fail (rvector);

    if (length > rvector->len) {
        int i;
        ptr_vector_maybe_expand (rvector, (length - rvector->len));

        // Do not use memset for the portable issue. NULL is not necessary bitwise zero.
        for (i = rvector->len; i < length; i++)
            rvector->pdata[i] = NULL;
    } 

    rvector->len = length;
}


void*
ptr_vector_remove_element (ptr_vector_t *vector, int  index_)
{
    real_ptr_vector_t *rvector = (real_ptr_vector_t *)vector;
    void *result;

    return_val_if_fail (rvector, NULL);

    return_val_if_fail (index_ < rvector->len, NULL);

    result = rvector->pdata[index_];

    if (index_ != rvector->len - 1)
        memmove (rvector->pdata + index_, rvector->pdata + index_ + 1,
                 sizeof (void *) * (rvector->len - index_ - 1));

    rvector->len -= 1;

    rvector->pdata[rvector->len] = NULL;

    return result;
}


ptr_vector_t*
ptr_vector_remove_elements (ptr_vector_t *vector, int  index_, int length)
{
    real_ptr_vector_t *rvector = (real_ptr_vector_t *)vector;
    int i;

    return_val_if_fail (rvector != NULL, NULL);
    return_val_if_fail (index_ < rvector->len, NULL);
    return_val_if_fail (index_ + length <= rvector->len, NULL);

    if (index_ + length != rvector->len) {
        memmove (&rvector->pdata[index_],
                 &rvector->pdata[index_ + length],
                 (rvector->len - (index_ + length)) * sizeof (void *));
    }

    rvector->len -= length;


    for (i = 0; i < length; i++)
        rvector->pdata[rvector->len + i] = NULL;

    return vector;
}


bool
ptr_vector_remove (ptr_vector_t *vector, void *data)
{
    int i;

    return_val_if_fail (vector, false);

    for (i = 0; i < vector->len; i += 1) {
        if (vector->pdata[i] == data) {
            ptr_vector_remove_element (vector, i);
            return true;
        }
    }

    return false;
}


void
ptr_vector_append_element(ptr_vector_t *vector, void *data)
{
    real_ptr_vector_t *rvector = (real_ptr_vector_t *)vector;

    return_if_fail (rvector);

    ptr_vector_maybe_expand (rvector, 1);

    rvector->pdata[rvector->len++] = data;
}


void
ptr_vector_append_elements(ptr_vector_t *vector, void **data, int len)
{
	real_ptr_vector_t *rvector = (real_ptr_vector_t *)vector;

	return_if_fail(rvector);

	return_if_fail(len > 0);

	ptr_vector_maybe_expand(rvector, len);

	memmove(&rvector->pdata[rvector->len], data, sizeof(void*) * len);

	vector->len += len;
}


void
ptr_vector_sort (ptr_vector_t *vector, comp_func_t compare_func )
{
    return_if_fail (vector != NULL);

    qsort (vector->pdata, vector->len, sizeof (void *), compare_func);
}


bool
ptr_vector_empty (ptr_vector_t *vector)
{
    return_val_if_fail (vector != NULL, true);
    return (vector->len == 0);
}


bool
ptr_vector_is_in(ptr_vector_t *vector, void *data)
{
    int i;

    return_val_if_fail(vector, false);

    for (i = 0; i < vector->len; i += 1) {
        if (vector->pdata[i] == data) {
            return true;
        }
    }

    return false;
}


void 
ptr_vector_reverse_in_place(ptr_vector_t *v)
{
	int i, j;

	if (!v || v->len <= 1)
		return;

	for (i = 0, j = v->len - 1; i < j; ++i, --j) {
		void *tmp = ptr_vector_at(v, i);

		ptr_vector_at(v, i) = ptr_vector_at(v, j);

		ptr_vector_at(v, j) = tmp;
	}
}


void
ptr_vector_clone(ptr_vector_t *dest, ptr_vector_t *src)
{
    int i; 

    if (dest == NULL || src == NULL) {
        return;
    }

    ptr_vector_clear(dest, false);

    for (i = 0; i < src->len; i++) {
        ptr_vector_append_element(dest, ptr_vector_at(src, i));
    }
}