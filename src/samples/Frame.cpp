#include "Frame.hpp"

#include <memory.h>
#include <string>
#include <iostream>
#include <stdexcept>
#include <iterator>

void base::samples::frame::frame_attrib_t::set(const std::string& name, const std::string& data)
{
    name_ = name;
    data_ = data;
}

bool base::samples::frame::frame_size_t::operator==(const base::samples::frame::frame_size_t& other) const
{
    if(width == other.width && height==other.height)
        return true;
    return false;
}

bool base::samples::frame::frame_size_t::operator!=(const base::samples::frame::frame_size_t& other) const
{
    return !(*this == other);
}

base::samples::frame::Frame::Frame() : image(), size(), data_depth(0), pixel_size(0), frame_mode()
{
    setDataDepth(0);
    reset();
}

base::samples::frame::Frame::Frame(uint16_t width, uint16_t height, uint8_t depth, base::samples::frame::frame_mode_t mode, const uint8_t val, size_t sizeInBytes)
{
    init(width,height,depth,mode,val,sizeInBytes);
}

base::samples::frame::Frame::Frame(const base::samples::frame::Frame& other, bool bcopy)
{
    init(other,bcopy);
}

void base::samples::frame::Frame::copyImageIndependantAttributes(const base::samples::frame::Frame& other)
{
    std::vector<frame_attrib_t>::const_iterator iter = other.attributes.begin();
    for(;iter!= other.attributes.end();++iter)
        setAttribute(iter->name_,iter->data_);
    time = other.time;
    received_time = other.received_time;
    frame_status = other.frame_status;
}

void base::samples::frame::Frame::init(const base::samples::frame::Frame& other, bool bcopy)
{
    //hdr is copied by attributes = other.attributes;
    init(other.getWidth(),other.getHeight(),other.getDataDepth(), other.getFrameMode(),-1,other.getNumberOfBytes());
    if(bcopy)
        setImage(other.getImage());
    copyImageIndependantAttributes(other);
}

void base::samples::frame::Frame::init(uint16_t width, uint16_t height, uint8_t depth, base::samples::frame::frame_mode_t mode, const uint8_t val, size_t sizeInBytes)
{
    //change size if the frame does not fit
    if(this->size.height != height || this->size.width !=  width || this->frame_mode != mode || 
        this->data_depth != depth || (sizeInBytes != 0 && sizeInBytes != image.size()))
    {
    //check if depth = 0
    //this might be a programmer error 
    if(depth==0 && (height != 0 || width != 0 ))
        throw std::runtime_error("Frame::init: Cannot initialize frame with depth = 0.");

    this->frame_mode = mode;
    this->size = frame_size_t(width, height);
    setDataDepth(depth);
    }
    //calculate size if not given 
    if(!sizeInBytes)
        sizeInBytes = getPixelSize() * getPixelCount();

    validateImageSize(sizeInBytes);
    image.resize(sizeInBytes);
    reset(val);
}

void base::samples::frame::Frame::reset(const int val)
{
    this->time = base::Time();
    if (this->image.size() > 0 && val >= 0) {
        memset(&this->image[0], val%256, this->image.size());
    }
    setStatus(STATUS_EMPTY);
    attributes.clear();
}

void base::samples::frame::Frame::swap(base::samples::frame::Frame& frame)
{
    //swap vector
    image.swap(frame.image);
    attributes.swap(frame.attributes);

    //copy values to temp
    base::Time temp_time = frame.time;
    base::Time temp_received_time = frame.received_time;
    frame_size_t temp_size = frame.size;
    uint32_t temp_data_depth = frame.data_depth;
    uint32_t temp_pixel_size = frame.pixel_size;
    uint32_t temp_row_size = frame.row_size;
    frame_mode_t temp_frame_mode = frame.frame_mode;
    frame_status_t temp_frame_status = frame.frame_status;

    //copy values
    frame.time = time;
    frame.received_time = received_time;
    frame.size = size;
    frame.data_depth = data_depth;
    frame.pixel_size = pixel_size;
    frame.row_size = row_size;
    frame.frame_mode = frame_mode;
    frame.frame_status = frame_status;

    time = temp_time;
    received_time = temp_received_time;
    size = temp_size;
    data_depth = temp_data_depth;
    pixel_size = temp_pixel_size;
    row_size = temp_row_size;
    frame_mode = temp_frame_mode;
    frame_status = temp_frame_status;
}

bool base::samples::frame::Frame::isHDR() const
{
    return (hasAttribute("hdr")&&getAttribute<bool>("hdr"));
}

void base::samples::frame::Frame::setHDR(bool value)
{
    setAttribute<bool>("hdr",true);
}

bool base::samples::frame::Frame::isCompressed() const
{
    return frame_mode >= COMPRESSED_MODES;
}

bool base::samples::frame::Frame::isGrayscale() const
{
    return this->frame_mode == MODE_GRAYSCALE;
}

bool base::samples::frame::Frame::isRGB() const
{
    return this->frame_mode == MODE_RGB;
}

bool base::samples::frame::Frame::isBayer() const
{
    return (this->frame_mode == MODE_BAYER || this->frame_mode == MODE_BAYER_RGGB || this->frame_mode == MODE_BAYER_GRBG || this->frame_mode == MODE_BAYER_BGGR || this->frame_mode == MODE_BAYER_GBRG);
}

void base::samples::frame::Frame::setStatus(const base::samples::frame::frame_status_t value)
{
    frame_status = value;
}

base::samples::frame::frame_status_t base::samples::frame::Frame::getStatus() const
{
    return frame_status;
}

uint32_t base::samples::frame::Frame::getChannelCount() const
{
    return getChannelCount(this->frame_mode);
}

uint32_t base::samples::frame::Frame::getChannelCount(base::samples::frame::frame_mode_t mode)
{
    switch (mode)
    {
        case MODE_UNDEFINED:
            return 0;
        case MODE_BAYER:
        case MODE_BAYER_RGGB:
        case MODE_BAYER_BGGR:
        case MODE_BAYER_GBRG:
        case MODE_BAYER_GRBG:
        case MODE_GRAYSCALE:
        case MODE_UYVY:
            return 1;
        case MODE_RGB:
        case MODE_BGR:
            return 3;
        case MODE_RGB32:
            return 4;
        case MODE_PJPG:
        case MODE_JPEG:
        case MODE_PNG:
            return 1;
        default:
            throw std::runtime_error("Frame::getChannelCount: Unknown frame_mode");
            return 0;
    }
}

base::samples::frame::frame_mode_t base::samples::frame::Frame::toFrameMode(const std::string& str)
{
    if(str == "MODE_UNDEFINED")
        return MODE_UNDEFINED;
    else if (str == "MODE_GRAYSCALE")
        return MODE_GRAYSCALE;
    else if (str == "MODE_RGB")
        return MODE_RGB;
    else if (str == "MODE_BGR")
        return MODE_BGR;
    else if (str == "MODE_UYVY")
        return MODE_UYVY;
    else if (str == "RAW_MODES")
        return RAW_MODES;
    else if (str == "MODE_BAYER")
        return MODE_BAYER;
    else if (str == "MODE_BAYER_RGGB")
        return MODE_BAYER_RGGB;
    else if (str == "MODE_BAYER_GRBG")
        return MODE_BAYER_GRBG;
    else if (str == "MODE_BAYER_BGGR")
        return MODE_BAYER_BGGR;
    else if (str == "MODE_BAYER_GBRG")
        return MODE_BAYER_GBRG;
    else if (str == "MODE_RGB32")
        return MODE_RGB32;
    else if (str == "COMPRESSED_MODES")
        return COMPRESSED_MODES;
    else if (str == "MODE_PJPG")
        return MODE_PJPG;
    else if (str == "MODE_JPEG")
        return MODE_JPEG;
    else if (str == "MODE_PNG")
        return MODE_PNG;
    else
        return MODE_UNDEFINED;
}

base::samples::frame::frame_mode_t base::samples::frame::Frame::getFrameMode() const
{
    return this->frame_mode;
}

uint32_t base::samples::frame::Frame::getPixelSize() const
{
    return this->pixel_size;
}

uint32_t base::samples::frame::Frame::getRowSize() const
{
    if(isCompressed())
        throw std::runtime_error("Frame::getRowSize: There is no raw size for an compressed image!");
    return this->row_size;
}

uint32_t base::samples::frame::Frame::getNumberOfBytes() const
{
    return image.size();
}

uint32_t base::samples::frame::Frame::getPixelCount() const
{
    return size.width * size.height;
}

uint32_t base::samples::frame::Frame::getDataDepth() const
{
    return this->data_depth;
}

void base::samples::frame::Frame::setDataDepth(uint32_t value)
{
    this->data_depth = value;

    // Update pixel size
    uint32_t comp_size = ((this->data_depth + 7) / 8);
    this->pixel_size = getChannelCount(this->frame_mode) * comp_size;

    //update row size
    if(isCompressed())
        this->row_size = 0;                         //disable row size
    else
        this->row_size = this->pixel_size * getWidth();
}

void base::samples::frame::Frame::setFrameMode(base::samples::frame::frame_mode_t mode)
{
    this->frame_mode = mode;

    // Update pixel size
    uint32_t comp_size = ((this->data_depth + 7) / 8);
    this->pixel_size = getChannelCount(this->frame_mode) * comp_size;

    //update row size
    if(isCompressed())
        this->row_size = 0;                         //disable row size
    else
        this->row_size = this->pixel_size * getWidth();
}

base::samples::frame::frame_size_t base::samples::frame::Frame::getSize() const
{
    return this->size;
}

uint16_t base::samples::frame::Frame::getWidth() const
{
    return this->size.width;
}

uint16_t base::samples::frame::Frame::getHeight() const
{
    return this->size.height;
}

const std::vector< uint8_t >& base::samples::frame::Frame::getImage() const
{
    return this->image;
}

void base::samples::frame::Frame::validateImageSize(size_t sizeToValidate) const
{
    size_t expected_size = getPixelSize()*getPixelCount();
    if (!isCompressed() && sizeToValidate != expected_size){
        std::cerr << "Frame: "
                    << __FUNCTION__ << " (" << __FILE__ << ", line "
                    << __LINE__ << "): " << "image size mismatch in setImage() ("
                    << "getting " << sizeToValidate << " bytes but I was expecting " << expected_size << " bytes)"
                    << std::endl;
        throw std::runtime_error("Frame::validateImageSize: wrong image size!");
    }
}

void base::samples::frame::Frame::setImage(const std::vector< uint8_t >& newImage)
{
    // calling the overloading function wich uses the "char*" interface
    return setImage(newImage.data(), newImage.size());
}

void base::samples::frame::Frame::setImage(const char* data, size_t newImageSize)
{
    return setImage(reinterpret_cast<const uint8_t*>(data), newImageSize);
}

void base::samples::frame::Frame::setImage(const uint8_t* data, size_t newImageSize)
{
    validateImageSize(newImageSize);
    image.resize(newImageSize);
    memcpy(&this->image[0], data, newImageSize);
}

uint8_t* base::samples::frame::Frame::getImagePtr()
{
    return static_cast<uint8_t *>(image.data());
}

const uint8_t* base::samples::frame::Frame::getImageConstPtr() const
{
    return static_cast<const uint8_t *>(image.data());
}

uint8_t* base::samples::frame::Frame::getLastByte()
{
    return static_cast<uint8_t *>(&this->image.back());
}

const uint8_t* base::samples::frame::Frame::getLastConstByte() const
{
    return static_cast<const uint8_t *>(&this->image.back());
}

bool base::samples::frame::Frame::hasAttribute(const std::string& name) const
{
    std::vector<frame_attrib_t>::const_iterator _iter = attributes.begin();
    for (;_iter != attributes.end();_iter++)
    {
        if (_iter->name_ == name)
            return true;
    }
    return false;
}

bool base::samples::frame::Frame::deleteAttribute(const std::string& name)
{
    std::vector<frame_attrib_t>::iterator _iter = attributes.begin();
    for (;_iter != attributes.end();_iter++)
    {
        if (_iter->name_ == name)
        {
            attributes.erase(_iter);
            return true;
        }
    }
    return false;
}



















