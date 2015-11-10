/*! \file frame.h
    \brief container for imaging data 
*/

#ifndef BASE_SAMPLES_FRAME_H__
#define BASE_SAMPLES_FRAME_H__

#include <stdint.h>
#include <memory.h>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <iterator>

#include <base/Time.hpp>


namespace base { namespace samples { namespace frame { 
	struct frame_attrib_t
	{
	    std::string data_;
	    std::string name_;
	    inline void set(const std::string &name,const std::string &data)
	    {
		name_ = name;
		data_ = data;
	    }
	};

	struct frame_size_t {
	    frame_size_t() : width(0), height(0) {}
	    frame_size_t(uint16_t w, uint16_t h) : width(w), height(h) {}

            bool operator==(const frame_size_t &other) const
            {
              if(width == other.width && height==other.height)
                return true;
              return false;
            };

            bool operator!=(const frame_size_t &other) const
            {
              return !(*this == other);
            };
	    uint16_t width;
	    uint16_t height;
	};

	enum frame_mode_t {
	    MODE_UNDEFINED = 0,
	    MODE_GRAYSCALE = 1,
	    MODE_RGB       = 2,
	    MODE_UYVY	   = 3,
	    MODE_BGR	   = 4,
            MODE_RGB32     = 5,
	    RAW_MODES = 128,
	    MODE_BAYER = RAW_MODES + 0,
	    MODE_BAYER_RGGB = RAW_MODES + 1,
	    MODE_BAYER_GRBG = RAW_MODES + 2,
	    MODE_BAYER_BGGR = RAW_MODES + 3,
	    MODE_BAYER_GBRG = RAW_MODES + 4,
            COMPRESSED_MODES = 256,                      //if an image is compressed it has no relationship
                                                         //between number of pixels and number of bytes
	    MODE_PJPG = COMPRESSED_MODES + 1,
            MODE_JPEG = COMPRESSED_MODES + 2
	};

	enum frame_status_t {
	    STATUS_EMPTY,
	    STATUS_VALID, 
	    STATUS_INVALID
	};

	/* A single image frame */
	struct Frame
	{
	public:
	    /**
	     * Initialize the frame
	     * @param width the image width, in pixels
	     * @param height the image height, in pixels
	     * @param data_depth the number of effective bits per pixels
	     * @param mode the frame mode (raw, hdr, greyscale, colour)
	     */
	    Frame() :
		    image(), size(), data_depth(0), pixel_size(0),
		    frame_mode()
	    {
		setDataDepth(0);
		reset();
	    }
	    
            //@depth number of bits per pixel and channel
	    Frame(uint16_t width, uint16_t height, uint8_t depth=8, frame_mode_t mode=MODE_GRAYSCALE, uint8_t const val = 0,size_t sizeInBytes=0)
	    {
		init(width,height,depth,mode,val,sizeInBytes);
	    }

	    //makes a copy of other
	    Frame(const Frame &other,bool bcopy = true)
	    {
		init(other,bcopy);
	    }
	    
	    //copies all attributes which are independent from size and mode
            //if an attribute already exists the old value is over written
	    void copyImageIndependantAttributes(const Frame &other)
	    {
               std::vector<frame_attrib_t>::const_iterator iter = other.attributes.begin();
               for(;iter!= other.attributes.end();++iter)
                   setAttribute(iter->name_,iter->data_);
	       time = other.time;
	       received_time = other.received_time;
	       frame_status = other.frame_status;
	    }

	    //makes a copy of other
	    void init(const Frame &other,bool bcopy = true)
	    {
	       //hdr is copied by attributes = other.attributes;
	       init(other.getWidth(),other.getHeight(),other.getDataDepth(), other.getFrameMode(),-1,other.getNumberOfBytes());
	       if(bcopy)
		  setImage(other.getImage());
	       copyImageIndependantAttributes(other);
	    }

	    void init(uint16_t width, uint16_t height, uint8_t depth=8, frame_mode_t mode=MODE_GRAYSCALE, const uint8_t val = 0, size_t sizeInBytes=0)
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

            // if val is negative the image will not be initialized
	    void reset(int const val = 0)
	    {
		this->time = base::Time();
		if (this->image.size() > 0 && val >= 0) {
		    memset(&this->image[0], val%256, this->image.size());
		}
		setStatus(STATUS_EMPTY);
		attributes.clear();
	    }

            void swap(Frame &frame)
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

	    inline bool isHDR()const       
	    {
		return (hasAttribute("hdr")&&getAttribute<bool>("hdr"));
	    }

	    inline void setHDR(bool value)  
	    {
		setAttribute<bool>("hdr",true);
	    }

            inline bool isCompressed()const
            {
                return frame_mode >= COMPRESSED_MODES;
            }

	    inline bool isGrayscale()const {
		return this->frame_mode == MODE_GRAYSCALE;
	    }
	    inline bool isRGB()const       {
		return this->frame_mode == MODE_RGB;
	    }

            inline bool isBayer()const     {
                return (this->frame_mode == MODE_BAYER || this->frame_mode == MODE_BAYER_RGGB || this->frame_mode == MODE_BAYER_GRBG || this->frame_mode == MODE_BAYER_BGGR || this->frame_mode == MODE_BAYER_GBRG);
            }

	    inline void setStatus(const frame_status_t value){
		frame_status = value;
	    }

	    inline frame_status_t getStatus()const{
		return frame_status;
	    }

	    inline uint32_t getChannelCount() const {
		return getChannelCount(this->frame_mode);
	    }
	    static uint32_t getChannelCount(frame_mode_t mode)
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
                    return 1;
		default:
                    throw std::runtime_error("Frame::getChannelCount: Unknown frame_mode");
		    return 0;
		}
	    }

            //qt ruby does not support enums as slot parameters
            //therefore frame_mode_t is passed as string
            static frame_mode_t toFrameMode(const std::string &str)
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
              else
                return MODE_UNDEFINED;
            };

	    inline frame_mode_t getFrameMode() const {
		return this->frame_mode;
	    }

	    /**
	     * Returns the size of a pixel (in bytes). This takes into account the image
	     * mode as well as the data depth.
	     * @return Number of channels * bytes used to represent one colour
	     */
	    inline uint32_t getPixelSize() const {
		return this->pixel_size;
	    }
	    
	    /**
	     * Returns the size of a row (in bytes). This takes into account the image
	     * mode as well as the data depth.
	     * @return Number of channels * width * bytes used to represent one colour
             * @return 0 if the image is compressed
	     */
	    inline uint32_t getRowSize() const {
                if(isCompressed())
                    throw std::runtime_error("Frame::getRowSize: There is no raw size for an compressed image!");
		return this->row_size;
	    }

	     /**
	     * Returns the total number of bytes for the image
	     */
	    inline uint32_t getNumberOfBytes() const {
		return image.size();
	    }

	    /**
	     * Returns the total count of pixels in this frame
	     * @return Returns the overall number of pixels (width * height)
	     */
	    inline uint32_t getPixelCount() const {
		return size.width * size.height;
	    }

	    inline uint32_t getDataDepth() const {
		return this->data_depth;
	    }
	    void setDataDepth(uint32_t value)
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

            void setFrameMode(frame_mode_t mode)
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

	    inline frame_size_t getSize() const {
		return this->size;
	    }
	    inline uint16_t getWidth() const {
		return this->size.width;
	    }
	    inline uint16_t getHeight() const {
		return this->size.height;
	    }

	    inline const std::vector<uint8_t> &getImage() const {
		return this->image;
	    }

            void validateImageSize(size_t sizeToValidate) const {
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
	    inline void setImage(const std::vector<uint8_t> &newImage) {
                // calling the overloading function wich uses the "char*" interface
                return setImage(newImage.data(), newImage.size());
	    }
            /** This is for backward compatibility for the people that were
             * using the 'char' signature */
	    inline void setImage(const char *data, size_t newImageSize) {
                return setImage(reinterpret_cast<const uint8_t*>(data), newImageSize);
            }
	    inline void setImage(const uint8_t *data, size_t newImageSize) {
                validateImageSize(newImageSize);
                image.resize(newImageSize);
		memcpy(&this->image[0], data, newImageSize);
	    }

	    inline uint8_t *getImagePtr() {
		return static_cast<uint8_t *>(image.data());
	    }
	    inline const uint8_t *getImageConstPtr() const {
		return static_cast<const uint8_t *>(image.data());
	    }

            inline uint8_t* getLastByte(){
              return static_cast<uint8_t *>(&this->image.back());
            }

            inline const uint8_t* getLastConstByte()const{
              return static_cast<const uint8_t *>(&this->image.back());
            }

	    inline bool hasAttribute(const std::string &name)const
	    {
		std::vector<frame_attrib_t>::const_iterator _iter = attributes.begin();
		for (;_iter != attributes.end();_iter++)
		{
		    if (_iter->name_ == name)
		        return true;
		}
		return false;
	    }
	    template<typename T>
	    inline T getAttribute(const std::string &name)const
	    {
		static T default_value;
		std::stringstream strstr;
	
		std::vector<frame_attrib_t>::const_iterator _iter = attributes.begin();
		for (;_iter != attributes.end();_iter++)
		{
		    if (_iter->name_ == name)
		    {
		        T data;
			strstr << _iter->data_;
			strstr >> data;
			return data;
		    }
		}
		return default_value;
	    }

	    inline bool deleteAttribute(const std::string &name)
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
	    

	    template<typename T>
	    inline void setAttribute(const std::string &name,const T &data)
	    {
		//if attribute exists
		std::stringstream strstr;
		strstr << data;
		std::vector<frame_attrib_t>::iterator _iter = attributes.begin();
		for (;_iter != attributes.end();_iter++)
		{
		    if (_iter->name_ == name)
		    {
		        _iter->set(name,strstr.str());
		        return;
		    }
		}
		//if attribute does not exist
		attributes.push_back(frame_attrib_t());
		attributes.back().set(name,strstr.str());
		return ;
	    }
	    
	    template <typename Tp> Tp& at(unsigned int column,unsigned int row)
		{
	    	if(column >= size.width || row >= size.height )
	    		throw std::runtime_error("out of index");
	    	return *((Tp*)(getImagePtr()+row*getRowSize()+column*getPixelSize()));
		}

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

	    /** The number of effective bits per channel. The number
	     * of actual bits per channel is always a multiple of
	     * height (i.e. a 12-bit effective depth is represented
	     * using 16-bits per channels). The number of greyscale
	     * levels is 2^(this_number)
             */
	    uint32_t                data_depth;
            /** The size of one pixel, in bytes
             *
             * For instance, for a RGB image with a 8 bit data depth, it would
             * be 3. For a 12 bit non-packed image (i.e with each channel
             * encoded on 2 bytes), it would be 6.
             */
	    uint32_t                pixel_size;

            /** The size of a complete row in bytes
             */
	    uint32_t 		    row_size;

            /** The colorspace of the image
             */
	    frame_mode_t            frame_mode;

            /** Status flag */
	    frame_status_t	    frame_status;
	};

	struct FramePair
	{
	    base::Time time;
	    Frame first;
            Frame second;
	    uint32_t id;	
	};
}}}

#endif
