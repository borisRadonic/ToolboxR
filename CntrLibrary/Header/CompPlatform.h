#pragma once

#if defined (__CC_ARM) || defined(__GNUC__)
#include "cmsis_gcc.h"
#ifndef   __FORCEINLINE                 
#define __FORCEINLINE                   __attribute__((always_inline)) inline
#endif
#elif defined _MSC_VER
#define __FORCEINLINE __forceinline
#define __STATIC_FORCEINLINE static __forceinline
#define __FORCEINLINE inline
//#define __NO_RETURN
//#define __USED
//#define __WEAK
#else
#define __FORCEINLINE
#define __STATIC_FORCEINLINE static inline
#define __FORCEINLINE inline
#endif