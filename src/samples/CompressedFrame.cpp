#include "CompressedFrame.hpp"

base::samples::frame::frame_compressed_mode_t base::samples::frame::CompressedFrame::toFrameMode(const std::string& str)
{
    if(str == "MODE_COMPRESSED_UNDEFINED")
        return frame_compressed_mode_t::MODE_COMPRESSED_UNDEFINED;
    else if (str == "MODE_PJPG")
        return frame_compressed_mode_t::MODE_PJPG;
}
