#if __has_include("rice/rice.hpp")
#include "rice/rice.hpp"
#else
#include "rice/Class.hpp"
#endif
extern void Init_eigen_ext();

#ifdef SISL_FOUND
extern void Init_spline_ext();
#endif

extern "C" void Init_base_types_ruby()
{
    Init_eigen_ext();
#ifdef SISL_FOUND
    Init_spline_ext();
#endif
}

