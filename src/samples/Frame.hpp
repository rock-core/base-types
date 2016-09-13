/*! \file frame.h
    \brief container for imaging data 
*/

#ifndef BASE_SAMPLES_FRAME_H__
#define BASE_SAMPLES_FRAME_H__

#include <stdint.h>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <base/Time.hpp>


namespace base { namespace samples { namespace frame { 
	struct frame_attrib_t
	{
	    std::string data_;
	    std::string name_;
	    void set(const std::string &name,const std::string &data);
	};

	struct frame_size_t {
	    frame_size_t() : width(0), height(0) {}
	    frame_size_t(uint16_t w, uint16_t h) : width(w), height(h) {}

            bool operator==(const frame_size_t &other) const;
              

            bool operator!=(const frame_size_t &other) const;
               
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
            MODE_JPEG = COMPRESSED_MODES + 2,
            MODE_PNG = COMPRESSED_MODES + 3
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
	    Frame();		    
	    
            //@depth number of bits per pixel and channel
	    Frame(uint16_t width, uint16_t height, uint8_t depth=8, frame_mode_t mode=MODE_GRAYSCALE, uint8_t const val = 0,size_t sizeInBytes=0);

	    //makes a copy of other
	    Frame(const Frame &other,bool bcopy = true);
	    
	    //copies all attributes which are independent from size and mode
            //if an attribute already exists the old value is over written
	    void copyImageIndependantAttributes(const Frame &other);

	    //makes a copy of other
	    void init(const Frame &other,bool bcopy = true);

	    void init(uint16_t width, uint16_t height, uint8_t depth=8, frame_mode_t mode=MODE_GRAYSCALE, const uint8_t val = 0, size_t sizeInBytes=0);

            // if val is negative the image will not be initialized
	    void reset(int const val = 0);

            void swap(Frame &frame);

	    bool isHDR()const;

	    void setHDR(bool value);

            bool isCompressed()const;

	    bool isGrayscale()const;
		
	    bool isRGB()const;

            bool isBayer()const;

	    void setStatus(const frame_status_t value);

	    frame_status_t getStatus()const;

	    uint32_t getChannelCount() const;
            
	    static uint32_t getChannelCount(frame_mode_t mode);

            //qt ruby does not support enums as slot parameters
            //therefore frame_mode_t is passed as string
            static frame_mode_t toFrameMode(const std::string &str);

	    frame_mode_t getFrameMode() const;

	    /**
	     * Returns the size of a pixel (in bytes). This takes into account the image
	     * mode as well as the data depth.
	     * @return Number of channels * bytes used to represent one colour
	     */
	    uint32_t getPixelSize() const;
	    
	    /**
	     * Returns the size of a row (in bytes). This takes into account the image
	     * mode as well as the data depth.
	     * @return Number of channels * width * bytes used to represent one colour
             * @return 0 if the image is compressed
	     */
	    uint32_t getRowSize() const;

	     /**
	     * Returns the total number of bytes for the image
	     */
	    uint32_t getNumberOfBytes() const;

	    /**
	     * Returns the total count of pixels in this frame
	     * @return Returns the overall number of pixels (width * height)
	     */
	    uint32_t getPixelCount() const;

	    uint32_t getDataDepth() const;
		
	    void setDataDepth(uint32_t value);

            void setFrameMode(frame_mode_t mode);

	    frame_size_t getSize() const;

	    uint16_t getWidth() const;
            
	    uint16_t getHeight() const;

	    const std::vector<uint8_t> &getImage() const;

            void validateImageSize(size_t sizeToValidate) const;
            
	    void setImage(const std::vector<uint8_t> &newImage);
            
            /** This is for backward compatibility for the people that were
             * using the 'char' signature */
            void setImage(const char *data, size_t newImageSize);
               
	    void setImage(const uint8_t *data, size_t newImageSize);

	    uint8_t *getImagePtr();
            
	    const uint8_t *getImageConstPtr() const;

            uint8_t* getLastByte();

            const uint8_t* getLastConstByte()const;

	    bool hasAttribute(const std::string &name)const;
            
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

	    bool deleteAttribute(const std::string &name);	    

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
