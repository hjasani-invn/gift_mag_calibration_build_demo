#include "imm_gc.h"

#define my_malloc malloc
#define my_free free
#define my_realloc realloc

static intptr_t magic = 20160831;

struct header {
	struct header	*prev;
	struct header	*next;
	intptr_t		magic;
};

static void *get_true_malloced_ptr(void *p)
{
	return ((char *)p - sizeof(struct header));
}

static void *get_returned_ptr(void *p)
{
	return ((char *)p + sizeof(struct header));
}

#ifdef _MSC_VER
static __declspec(thread)
#else
static __thread
#endif
struct header thread_status = { 0 };


void gc_exit_and_destroy_everything()
{
	struct header *hd = thread_status.next;

	thread_status.next = NULL;
	thread_status.prev = NULL;
	thread_status.magic = (intptr_t)NULL;

	while (hd) {
		struct header tmp = *hd;

		if (hd->magic != magic)
			exit(100);

		hd->magic = 0;

		my_free(hd);

		hd = tmp.next;
	}
}

void* gc_malloc(intptr_t sz)
{
	struct header *mem = NULL;
	void *ret = NULL;

	if (sz <= 0)
		goto on_error;

	mem = (struct header *)my_malloc(sizeof(struct header) + sz);

	if (!mem)
		goto on_error;

	if (thread_status.magic == 0) {
		thread_status.magic = magic;
		thread_status.prev = NULL;
		thread_status.next = NULL;
	}
	
	mem->magic = magic;
	mem->prev = &thread_status;

	if (thread_status.next != NULL) {		
		thread_status.next->prev = mem;
	}

	mem->next = thread_status.next;
	thread_status.next = mem;

	ret = get_returned_ptr(mem);

	return ret;

on_error:
	gc_exit_and_destroy_everything();
	exit(99);
}

void* gc_realloc(void *p, intptr_t sz)
{
	struct header *mem = NULL;
    void *ret = NULL;

	if (p) {
		struct header *old_hd = (struct header *)get_true_malloced_ptr(p);

		if (old_hd->magic != magic)
			goto on_error;

		if (sz < 0)
			goto on_error;

		mem = (struct header*)my_realloc(old_hd, sizeof(struct header) + sz);

		if (!mem)
			goto on_error;

		mem->next->prev = mem;
		mem->prev->next = mem;
		mem->magic = magic;

		ret = get_returned_ptr(mem);
	}
	else
		return gc_malloc(sz);

    return ret;

on_error:
	gc_exit_and_destroy_everything();
	exit(99);
}

void gc_free(void *p)
{
	struct header *mem = NULL;

	if (p) {
		mem = (struct header *)get_true_malloced_ptr(p);

		if (mem->magic != magic)
			goto on_error;
		else {
			struct header tmp = *mem;

			struct header *prev = tmp.prev;
			struct header *next = tmp.next;

			mem->magic = 0;
			mem->next = NULL;
			mem->prev = NULL;

			my_free(mem);

			tmp.prev->next = next;
			tmp.next->prev = prev;
		}
	}

	return;

on_error:
	gc_exit_and_destroy_everything();
	exit(99);
}
