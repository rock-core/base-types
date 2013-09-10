#include <base/TimeMark.hpp>
#include <iostream>
#include "bench_func.h"

int main()
{
    const int count = 100000000;
    {
	base::TimeMark t("Float Aligned Vector3");
	for( int i=0; i<count; i++ )
	    mult_tv1(Vector3FloatAlign::Zero(), TransformFloatAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Float Unaligned Vector3");
	for( int i=0; i<count; i++ )
	    mult_tv2(Vector3FloatNoAlign::Zero(), TransformFloatNoAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Float Aligned Vector4");
	for( int i=0; i<count; i++ )
	    mult_tv3(Vector4FloatAlign::Zero(), TransformFloatAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Float Unaligned Vector4");
	for( int i=0; i<count; i++ )
	    mult_tv4(Vector4FloatNoAlign::Zero(), TransformFloatNoAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Aligned Vector3");
	for( int i=0; i<count; i++ )
	    mult_tv5(Vector3DoubleAlign::Zero(), TransformDoubleAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Unaligned Vector3");
	for( int i=0; i<count; i++ )
	    mult_tv6(Vector3DoubleNoAlign::Zero(), TransformDoubleNoAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Aligned Vector4");
	for( int i=0; i<count; i++ )
	    mult_tv7(Vector4DoubleAlign::Zero(), TransformDoubleAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Unaligned Vector4");
	for( int i=0; i<count; i++ )
	    mult_tv8(Vector4DoubleNoAlign::Zero(), TransformDoubleNoAlign::Identity());
	std::cerr << t << std::endl;
    }

    {
	base::TimeMark t("Float Aligned Transform");
	for( int i=0; i<count; i++ )
	    mult_tt1(TransformFloatAlign::Identity(), TransformFloatAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Float Unaligned Transform");
	for( int i=0; i<count; i++ )
	    mult_tt2(TransformFloatNoAlign::Identity(), TransformFloatNoAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Aligned Transform");
	for( int i=0; i<count; i++ )
	    mult_tt3(TransformDoubleAlign::Identity(), TransformDoubleAlign::Identity());
	std::cerr << t << std::endl;
    }
    {
	base::TimeMark t("Double Unaligned Transform");
	for( int i=0; i<count; i++ )
	    mult_tt4(TransformDoubleNoAlign::Identity(), TransformDoubleNoAlign::Identity());
	std::cerr << t << std::endl;
    }
}

