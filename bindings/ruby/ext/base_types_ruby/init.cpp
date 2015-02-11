#include "rice/Class.hpp"
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

