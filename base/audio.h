#ifndef BASE_SAMPLES_AUDIO_H__
#define BASE_SAMPLES_AUDIO_H__ 

#include <base/time.h>
#include <vector>

namespace base { namespace samples {
    struct AudioSamples
    {
	std::vector<float> samples;
	double samplingRate;
        Time time;
    };
}} 

#endif

