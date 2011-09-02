#ifndef _COMPRESSED_FRAME_H_
#define _COMPRESSED_FRAME_H_

#include <base/samples/frame.h>


namespace base { namespace samples { namespace frame { 
	enum frame_compressed_mode_t {
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

	    frame_compressed_mode_t            frame_mode;

            /** Status flag */
	    frame_status_t	    frame_status;
            
            static frame_compressed_mode_t toFrameMode(const std::string &str)
            {
              if(str == "MODE_COMPRESSED_UNDEFINED")
                return MODE_COMPRESSED_UNDEFINED;
              else if (str == "MODE_PJPG")
                return MODE_PJPG;
            }
        };


}}};

#endif
