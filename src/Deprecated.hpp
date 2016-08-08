#ifndef BASE_TYPES_DEPRECATED
#ifdef __GNUC__
    #define BASE_TYPES_DEPRECATED __attribute__ ((deprecated))
#else
    #define BASE_TYPES_DEPRECATED
#endif 
#endif

#ifndef BASE_TYPES_DEPRECATED_SUPPRESS_START
#ifdef __GNUC__
    #define BASE_TYPES_DEPRECATED_SUPPRESS_START _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#else
    #define BASE_TYPES_DEPRECATED_SUPPRESS_START
#endif 
#endif

#ifndef BASE_TYPES_DEPRECATED_SUPPRESS_STOP
#ifdef __GNUC__
    #define BASE_TYPES_DEPRECATED_SUPPRESS_STOP _Pragma("GCC diagnostic pop")
#else
    #define BASE_TYPES_DEPRECATED_SUPPRESS_STOP
#endif 
#endif