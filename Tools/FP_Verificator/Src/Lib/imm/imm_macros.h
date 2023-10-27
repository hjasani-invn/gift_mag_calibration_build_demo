#ifndef _IMM_MACROS_H_
#define _IMM_MACROS_H_

/* Guard C code in headers, while including them from C++ */
#ifdef  __cplusplus
#define IMM_BEGIN_DECLS  extern "C" {
#define IMM_END_DECLS    }
#else
#define IMM_BEGIN_DECLS
#define IMM_END_DECLS
#endif

#ifdef WIN32
#include <stdio.h>
#else
#define printf(...)
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

#ifndef	FALSE
#define	FALSE	(0)
#endif

#ifndef	TRUE
#define	TRUE	(!FALSE)
#endif

#undef	MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#undef	MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#undef	ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))

#define FLT_EPSILON       1.192093e-007
#define DBL_EPSILON       2.2204460492503131e-016

#define NEAR_ZERO(val)    (((val) > -FLT_EPSILON) && ((val) < FLT_EPSILON))
#define NEAR_EQUAL(a, b)  NEAR_ZERO((a) - (b))


#endif
