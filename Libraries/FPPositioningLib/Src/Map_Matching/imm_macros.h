#ifndef _IMM_MACROS_H_
#define _IMM_MACROS_H_

#include <float.h>

/* Guard C code in headers, while including them from C++ */
#ifdef  __cplusplus
#define IMM_BEGIN_DECLS  extern "C" {
#define IMM_END_DECLS    }
#else
#define IMM_BEGIN_DECLS
#define IMM_END_DECLS
#endif



#if !(defined (IMM_STMT_START) && defined (IMM_STMT_END))
#define IMM_STMT_START  do
#define IMM_STMT_END    while (0)
#endif

#define return_val_if_fail(expr,val)	            \
	IMM_STMT_START {			                    \
	if (expr) { } else {						    \
	return (val);							        \
	};				                                \
}IMM_STMT_END

#define return_if_fail(expr)	                    \
	IMM_STMT_START {			                    \
	if (expr) { } else {						    \
	return;							                \
	};				                                \
}IMM_STMT_END


#ifndef NULL
#  ifdef __cplusplus
#  define NULL        (0L)
#  else /* !__cplusplus */
#  define NULL        ((void*) 0)
#  endif /* !__cplusplus */
#endif


#if defined(WIN32) && !defined(EXTERN_SYMBOL)
#define EXTERN_SYMBOL __declspec(dllexport)
#elif !defined(EXTERN_SYMBOL)
#define EXTERN_SYMBOL extern
#endif



//#if !defined(__cplusplus) || (__cplusplus < 201103L)
//#define nullptr NULL
//#endif


#undef	MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#undef	MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#undef	ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))



#define NEAR_ZERO(val)    (((val) > -FLT_EPSILON) && ((val) < FLT_EPSILON))
#define NEAR_EQUAL(a, b)  NEAR_ZERO((a) - (b))


#endif
