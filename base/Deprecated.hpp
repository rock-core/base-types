#ifndef BASE_TYPES_DEPRECATED
#ifdef __GNUC__
    #define BASE_TYPES_DEPRECATED __attribute__ ((deprecated))
#else
    #define BASE_TYPES_DEPRECATED
#endif 
#endif

