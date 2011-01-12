#include "rice/Class.hpp"
extern void Init_eigen_ext();
extern void Init_spline_ext(Rice::Module& base_m);

extern "C" void Init_base_types_ext()
{
    Rice::Module rb_mTypes = Rice::define_module("Types");
    Rice::Module rb_mBase = Rice::define_module_under(rb_mTypes, "Base");
    Rice::Module rb_mGeometry = Rice::define_module_under(rb_mBase, "Geometry");

    Init_eigen_ext();
    Init_spline_ext(rb_mGeometry);
}

