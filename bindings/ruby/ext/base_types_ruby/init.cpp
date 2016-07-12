#include "rice/Class.hpp"

#ifdef SISL_FOUND
extern void Init_spline_ext();
#endif

extern "C" void Init_base_types_ruby()
{
#ifdef SISL_FOUND
    Init_spline_ext();
#endif
}

