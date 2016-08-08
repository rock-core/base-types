#ifndef _COMPRESSED_FRAME_H_
#define _COMPRESSED_FRAME_H_

#include <base/samples/Frame.hpp>

namespace base { namespace samples { namespace frame { 
	enum class frame_compressed_mode_t {
	    MODE_COMPRESSED_UNDEFINED = 0,
	    MODE_PJPG = 1	
	};

	struct CompressedFrame{

	    /** The time at which this frame has been captured
             *
             * This is obviously an estimate
             */
	    base::Time              time;
            /** The time at which this frame has been received on the system */
	    base::Time              received_time;

            /** The raw data */
	    std::vector<uint8_t>    	image;
            /** Additional metadata */
	    std::vector<frame_attrib_t> attributes;

	    /** The image size in pixels */
	    frame_size_t            size;

	    frame_compressed_mode_t frame_mode;

            /** Status flag */
	    frame_status_t	    frame_status;
            
            static frame_compressed_mode_t toFrameMode(const std::string &str);
        };


}}};

#endif
