#ifndef AEESTDDEF_H
#define AEESTDDEF_H
/*
=======================================================================

FILE:         AEEStdDef.h

DESCRIPTION:  definition of basic types, constants,
                 preprocessor macros

=======================================================================
*/
/*==============================================================================
  Copyright (c) 2005,2007,2012-2013 Qualcomm Technologies, Inc.
  All rights reserved. Qualcomm Proprietary and Confidential.
==============================================================================*/

#include <stdint.h>

#if defined(COMDEF_H) /* guards against a known re-definer */
#define _BOOLEAN_DEFINED
#define _UINT32_DEFINED
#define _UINT16_DEFINED
#define _UINT8_DEFINED
#define _INT32_DEFINED
#define _INT16_DEFINED
#define _INT8_DEFINED
#define _UINT64_DEFINED
#define _INT64_DEFINED
#define _BYTE_DEFINED
#endif /* #if !defined(COMDEF_H) */

/* -----------------------------------------------------------------------
** Standard Types
** ----------------------------------------------------------------------- */

/* The following definitions are the same accross platforms.  This first
** group are the sanctioned types.
*/
#ifndef _BOOLEAN_DEFINED
typedef  unsigned char      boolean;     /* Boolean value type. */
#define _BOOLEAN_DEFINED
#endif

#ifndef _UINT32_DEFINED
typedef  uint32_t           uint32;      /* Unsigned 32 bit value */
#define _UINT32_DEFINED
#endif

#ifndef _UINT16_DEFINED
typedef  unsigned short     uint16;      /* Unsigned 16 bit value */
#define _UINT16_DEFINED
#endif

#ifndef _UINT8_DEFINED
typedef  unsigned char      uint8;       /* Unsigned 8  bit value */
#define _UINT8_DEFINED
#endif

#ifndef _INT32_DEFINED
typedef  int32_t            int32;       /* Signed 32 bit value */
#define _INT32_DEFINED
#endif

#ifndef _INT16_DEFINED
typedef  signed short       int16;       /* Signed 16 bit value */
#define _INT16_DEFINED
#endif

#ifndef _INT8_DEFINED
typedef  signed char        int8;        /* Signed 8  bit value */
#define _INT8_DEFINED
#endif

#if defined(__GNUC__)
#define __int64 long long
#endif

#ifndef _UINT64_DEFINED
typedef  unsigned __int64   uint64;      /* Unsigned 64 bit value */
#define _UINT64_DEFINED
#endif

#ifndef _INT64_DEFINED
typedef  __int64            int64;       /* Signed 64 bit value */
#define _INT64_DEFINED
#endif

#ifndef _BYTE_DEFINED
typedef  unsigned char      byte;        /* byte type */
#define  _BYTE_DEFINED
#endif


#ifndef _AEEUID_DEFINED
typedef uint32             AEEUID;
#define _AEEUID_DEFINED
#endif

#ifndef _AEEIID_DEFINED
typedef uint32             AEEIID;
#define _AEEIID_DEFINED
#endif

#ifndef _AEECLSID_DEFINED
typedef uint32             AEECLSID;
#define _AEECLSID_DEFINED
#endif

#ifndef _AEEPRIVID_DEFINED
typedef uint32             AEEPRIVID;
#define _AEEPRIVID_DEFINED
#endif

#ifndef _AECHAR_DEFINED
typedef uint16             AECHAR;
#define _AECHAR_DEFINED
#endif

#ifndef _AEERESULT_DEFINED
typedef int                AEEResult;
#define _AEERESULT_DEFINED
#endif

/* -----------------------------------------------------------------------
** Function Calling Conventions
** ----------------------------------------------------------------------- */

#ifndef CDECL
#ifdef _MSC_VER
#define CDECL __cdecl
#else
#define CDECL
#endif /* _MSC_VER */
#endif /* CDECL */

/* -----------------------------------------------------------------------
** Constants
** ----------------------------------------------------------------------- */

#ifndef TRUE
#define TRUE   1   /* Boolean true value. */
#endif

#ifndef FALSE
#define FALSE  0   /* Boolean false value. */
#endif

#ifndef NULL
#define NULL  0
#endif

#ifndef MIN_INT8
#define MIN_INT8 -128
#endif
#ifndef MIN_INT16
#define MIN_INT16 -32768
#endif
#ifndef MIN_INT32
#define MIN_INT32 (~0x7fffffff)   /* -2147483648 is unsigned */
#endif
#ifndef MIN_INT64
#define MIN_INT64 (~0x7fffffffffffffffLL) /* -9223372036854775808 is unsigned */
#endif

#ifndef MAX_INT8
#define MAX_INT8 127
#endif
#ifndef MAX_INT16
#define MAX_INT16 32767
#endif
#ifndef MAX_INT32
#define MAX_INT32 2147483647
#endif
#ifndef MAX_INT64
#define MAX_INT64 9223372036854775807LL
#endif

#ifndef MAX_UINT8
#define MAX_UINT8 255
#endif
#ifndef MAX_UINT16
#define MAX_UINT16 65535
#endif
#ifndef MAX_UINT32
#define MAX_UINT32 4294967295u
#endif
#ifndef MAX_UINT64
#define MAX_UINT64 18446744073709551615uLL
#endif

#ifndef MIN_AECHAR
#define MIN_AECHAR 0
#endif
#ifndef MAX_AECHAR
#define MAX_AECHAR 65535
#endif


/* -----------------------------------------------------------------------
** Preprocessor helpers
** ----------------------------------------------------------------------- */
#define __STR__(x) #x
#define __TOSTR__(x) __STR__(x)
#define __FILE_LINE__ __FILE__ ":" __TOSTR__(__LINE__)

/* -----------------------------------------------------------------------
** Types for code generated from IDL
** ----------------------------------------------------------------------- */

#ifndef __QIDL_WCHAR_T_DEFINED__
#define __QIDL_WCHAR_T_DEFINED__
typedef uint16 _wchar_t;
#endif

/* __STRING_OBJECT__ will be deprecated in the future */
#if !defined(__QIDL_STRING_OBJECT_DEFINED__) && !defined(__STRING_OBJECT__)
#define __QIDL_STRING_OBJECT_DEFINED__
#define __STRING_OBJECT__
typedef struct _cstring_s {
   char* data;
   int dataLen;
   int dataLenReq;
} _cstring_t;

typedef struct _wstring_s {
   _wchar_t* data;
   int dataLen;
   int dataLenReq;
} _wstring_t;
#endif /* __QIDL_STRING_OBJECT_DEFINED__ */

#endif /* #ifndef AEESTDDEF_H */

