#ifndef _COMPRESSED_FRAME_H_
#define _COMPRESSED_FRAME_H_

#include <base/samples/frame.h>


namespace base { namespace samples { namespace frame { 

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

	    frame_mode_t            frame_mode;

        /** Status flag */
	    frame_status_t	    frame_status;
#if 0
		#ifndef __orogen
		std::vector<uint8_t> toRGB8() const{
			std::vector<uint8_t> data;
			data.resize(size.width*size.height);

			
		}
		#endif
#endif
	};


}}};

#endif
