
/*
    Utils_General.h

	General useful functions for dsp programming
*/


#pragma once

#include <climits>
#include <cmath>

#if defined(_MSC_VER)
#pragma once
#pragma warning(4 : 4250)          // "Inherits via dominance" 
#endif

namespace utils
{

	//===================| Constants |===================

	// PI
	static const double  PI = 3.1415926535897932384626433832795;

	// FORCEINLINE
    #if defined(_MSC_VER)
    #define FORCEINLINE __forceinline
    #else
    #define FORCEINLINE inline
    #endif



    //===================| Fixed Point Math |===================

     /*----- 16-bit signed -----*/
#if defined(_MSC_VER)
    typedef __int16                 Int16;
#elif (defined(__MWERKS__) || defined(__GNUC__) || defined(__BEOS__)) && SHRT_MAX == 0x7FFF
    typedef short int               Int16;
#else
#error No signed 16-bit integer type defined for this compiler!
#endif

    /*----- 32-bit signed -----*/
#if defined(_MSC_VER)
    typedef __int32                 Int32;
#elif (defined(__MWERKS__) || defined(__GNUC__) || defined(__BEOS__)) && INT_MAX == 0x7FFFFFFFL
    typedef int                     Int32;
#else
#error No signed 32-bit integer type defined for this compiler!
#endif

    /*----- 64-bit signed -----*/
#if defined(_MSC_VER)
    typedef __int64                 Int64;
#elif defined(__MWERKS__) || defined(__GNUC__)
    typedef long long               Int64;
#elif defined(__BEOS__)
    typedef int64                   Int64;
#else
#error No 64-bit integer type defined for this compiler!
#endif

    /*----- 32-bit unsigned ----*/
#if defined(_MSC_VER)
    typedef unsigned __int32        UInt32;
#elif (defined(__MWERKS__) || defined(__GNUC__) || defined(__BEOS__)) && UINT_MAX == 0xFFFFFFFFUL
    typedef unsigned int            UInt32;
#else
#error No unsigned 32-bit integer type defined for this compiler!
#endif

	// Helpers for fixed-point math
    union Fixed3232
    {
        Int64  _all;
        class
        {
        public:
#if defined(__POWERPC__)            /* big-endian */
            Int32   _msw;
            UInt32  _lsw;
#else                               /* little-endian */
            UInt32  _lsw;
            Int32   _msw;
#endif
        } _part;
    }; 

     // simple branch-free min/max 
    template <typename T> FORCEINLINE T min(T a, T b) { return (a < b) ? a : b; }
    template <typename T> FORCEINLINE T max(T a, T b) { return (a > b) ? a : b; }

    // IEEE-safe rounding 
    FORCEINLINE int  round_int(double x) { return (x >= 0.0) ? int(x + 0.5) : int(x - 0.5); }
    FORCEINLINE long round_long(double x) { return (x >= 0.0) ? long(x + 0.5) : long(x - 0.5); }

    // bidirectional (signed) bit-shift helper
    template <typename T>
    FORCEINLINE T shift_bidi(T x, int s)
    {
        return (s >= 0) ? (x << s) : (x >> (-s));
    }



} // namespace utils