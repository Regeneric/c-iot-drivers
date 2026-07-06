#pragma once

#include <stdint.h>
#include <stddef.h>

#ifndef true
    #define true         1
#endif
#ifndef false
    #define false        0
#endif

#define FOREVER          true
#define ALWAYS           true

#define ERROR            1
#define OK               0

#define ARRAY_LEN(arr)  (sizeof(arr) / sizeof((arr)[0]))

// Standard unsigned types
typedef uint8_t             uint8;
typedef uint16_t            uint16;
typedef uint32_t            uint32;
typedef uint64_t            uint64;

// Volatile unsigned types
// Standard unsigned types
typedef volatile uint8_t    vuint8;
typedef volatile uint16_t   vuint16;
typedef volatile uint32_t   vuint32;
typedef volatile uint64_t   vuint64;

// Best runtime performance - heavier on RAM 
typedef uint_fast8_t        fuint8;
typedef uint_fast16_t       fuint16;
typedef uint_fast32_t       fuint32;
typedef uint_fast64_t       fuint64;

// Easier on RAM - worse runtime performance
typedef uint_least8_t       luint8;
typedef uint_least16_t      luint16;
typedef uint_least32_t      luint32;
typedef uint_least64_t      luint64;   


// Standard signed types
typedef int8_t              int8;
typedef int16_t             int16;
typedef int32_t             int32;
typedef int64_t             int64;

// Volatile signed types
typedef volatile int8_t     vint8;
typedef volatile int16_t    vint16;
typedef volatile int32_t    vint32;
typedef volatile int64_t    vint64;

// Best runtime performance - heavier on RAM 
typedef int_fast8_t         fint8;
typedef int_fast16_t        fint16;
typedef int_fast32_t        fint32;
typedef int_fast64_t        fint64;

// Easier on RAM - worse runtime performance
typedef int_least8_t        lint8;
typedef int_least16_t       lint16;
typedef int_least32_t       lint32;
typedef int_least64_t       lint64;


typedef float               float32;
typedef double              float64;

typedef int8_t              bool8;
typedef int32_t             bool32;

typedef const char*         cstring;

#ifdef __cplusplus
    #define STATIC_ASSERT static_assert
#else
    #define STATIC_ASSERT _Static_assert
#endif

STATIC_ASSERT(sizeof(uint8)  == 1, "Expected u8 to be 1 byte");
STATIC_ASSERT(sizeof(uint16) == 2, "Expected u16 to be 2 bytes");
STATIC_ASSERT(sizeof(uint32) == 4, "Expected u32 to be 4 bytes");
STATIC_ASSERT(sizeof(uint64) == 8, "Expected u64 to be 8 bytes");

STATIC_ASSERT(sizeof(int8)  == 1, "Expected i8 to be 1 byte");
STATIC_ASSERT(sizeof(int16) == 2, "Expected i16 to be 2 bytes");
STATIC_ASSERT(sizeof(int32) == 4, "Expected i32 to be 4 bytes");
STATIC_ASSERT(sizeof(int64) == 8, "Expected i64 to be 8 bytes");

STATIC_ASSERT(sizeof(float32) == 4, "Expected f32 to be 4 bytes");

#if __SIZEOF_DOUBLE__ != 8
    #ifdef _MSC_VER
        #pragma message "WARNING: `double` is less than 8 bytes!"
    #else
        #warning WARNING: `double` is less than 8 bytes!
    #endif
#else
    STATIC_ASSERT(sizeof(float64) == 8, "Expected f64 to be 8 bytes");
#endif