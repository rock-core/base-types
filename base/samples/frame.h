/*! \file frame.h
    \brief container for imaging data 
*/

#ifndef BASE_SAMPLES_FRAME_H__
#define BASE_SAMPLES_FRAME_H__

#ifndef __orogen
#include <stdint.h>
#include <memory.h>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <iterator>
#endif

#include "base/time.h"


namespace base { namespace samples { namespace frame { 
	struct frame_attrib_t
	{
	    std::string data_;
	    std::string name_;
	    #ifndef __orogen
	    inline void set(const std::string &name,const std::string &data)
	    {
		name_ = name;
		data_ = data;
	    }
	    #endif
	};

	struct frame_size_t {
	#ifndef __orogen
	    frame_size_t() : width(0), height(0) {}
	    frame_size_t(uint16_t w, uint16_t h) : width(w), height(h) {}

            bool operator==(const frame_size_t &other)
            {
              if(width == other.width && height==other.height)
                return true;
              return false;
            };

            bool operator!=(const frame_size_t &other)
            {
              return !(*this == other);
            };
	#endif
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
	    MODE_PJPG = RAW_MODES 	+ 5
	};


	enum frame_status_t {
	    STATUS_EMPTY,
	    STATUS_VALID, 
	    STATUS_INVALID
	};

	#ifndef __orogen
	template<int pixel_size> struct PixelTraits;
	template<>
	struct PixelTraits<8>
	{
	    typedef uint8_t channel_t;
	};

	template<>
	struct PixelTraits<16>
	{
	    typedef uint16_t channel_t;
	};
	#endif
	
	#ifndef __orogen

	//iterates over one specific column
	//this is index save
        //at the moment only 8 Bit data depth is supported 
	class ConstColumnIterator: public std::iterator<std::input_iterator_tag, char>
	{
	  private:
	    uint32_t row_size;
	    const uint8_t* pdata;
	    const uint8_t* pend;
	    const uint8_t* pstart;
	  public:
          ConstColumnIterator(const ConstColumnIterator &other )
	    :row_size(other.row_size),pdata(other.pdata),pend(other.pend),pstart(other.pstart){};

	  ConstColumnIterator(uint32_t _row_size,const uint8_t* _pdata, const uint8_t* _pend)
	    :row_size(_row_size),pdata(_pdata),pend(_pend),pstart(_pdata){};

	  ConstColumnIterator()  //end iterator
	  {row_size =0;pdata = NULL;pend = NULL;pstart=NULL;};
	  
	  ConstColumnIterator &operator++()
	  {
	    pdata += row_size;
	    if(pdata > pend)
	    {
	      pdata = NULL;
	      pend = NULL;
              pstart = NULL;
	    }
	    return *this;
	  }
          
          ConstColumnIterator &operator--()
	  {
	    pdata -= row_size;
	    if(pdata < pstart)
	    {
	      pdata = NULL;
	      pend = NULL;
              pstart = NULL;
	    }
	    return *this;
	  }

          ConstColumnIterator& operator+=(unsigned int rows)
	  {
            //calc new position 
            pdata += row_size*rows;
	    if(pdata > pend)
	    {
	      pdata = NULL;
	      pend = NULL;
              pstart = NULL;
	    }
	    return *this;
	  }

          //calc distance between to iterators
          int operator-(const ConstColumnIterator &other)const
          {
            if(pend == NULL)
            {
              if(other.pend != NULL)
                return (other.pend -other.pdata)/row_size; 
              else
                return 0;
            }

            //check if iterator belongs to the same column
            if(pend != other.pend)
              throw std::runtime_error("Iterator mismatch. Iterators belong to different columns!");

            if(pdata < other.pdata)
              return -(other.pdata-pdata)/row_size;

            return (pdata-other.pdata)/row_size;
          }

          const ConstColumnIterator operator+(unsigned int rows)const
	  {
            //make a copy
            ConstColumnIterator result(*this);
            result += rows;
	    return result;
	  }

	  bool operator==(const ConstColumnIterator &other)const{ return (pdata == other.pdata);}
	  uint8_t operator*()const{return *pdata;}
	  bool operator==(uint8_t* p)const{return (pdata == p);}
	  bool operator!=(uint8_t* p)const{return !(pdata == p);}
	  bool operator!=(const ConstColumnIterator &other)const{return !(pdata == other.pdata);}
	  void operator=(const ConstColumnIterator &other)
	  {row_size = other.row_size;pdata = other.pdata;pend = other.pend;pstart = other.pstart;}
	};
	#endif
	
	/* A single image frame */
	struct Frame
	{
	#ifndef __orogen
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
	    Frame(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, uint8_t const val = 0, bool hdr = false)
	    {
		init(width,height,depth,mode,val,hdr);
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
	       init(other.getWidth(),other.getHeight(),other.getDataDepth(), other.getFrameMode(),-1);
	       if(bcopy)
		  setImage(other.getImage());
	       copyImageIndependantAttributes(other);
	    }
	    
	    void init(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, int const val = 0, bool hdr = false)
	    {
               //change size if the frame does not fit
	       if(size.height != height || size.width !=  width || this->frame_mode != frame_mode || 
                 this->data_depth != depth)
               {
                //check if depth = 0
                //this might be a programmer error 
                if(depth==0 && (height != 0 || width != 0 ))
                    throw std::runtime_error("Frame::init: Cannot initialize frame with depth = 0.");

		this->frame_mode = mode;
		this->size = frame_size_t(width, height);
		setDataDepth(depth);
		image.resize(getPixelSize() * getPixelCount());
               }
	       reset(val);
	       if(hdr)
	        setAttribute<bool>("hdr",true);
               else
	        setAttribute<bool>("hdr",false);
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

	    inline int getChannelCount() const {
		return getChannelCount(this->frame_mode);
	    }
	    static int getChannelCount(frame_mode_t mode)
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
				return 1; //Only for memoy allocation size is dynamic
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
              else if (str == "MODE_PJPG")
                return MODE_PJPG;
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
	    inline int getPixelSize() const {
		return this->pixel_size;
	    }
	    
	    /**
	     * Returns the size of a row (in bytes). This takes into account the image
	     * mode as well as the data depth.
	     * @return Number of channels * width * bytes used to represent one colour
	     */
	    inline int getRowSize() const {
		return this->row_size;
	    }

	     /**
	     * Returns the total count of pixels in this frame * data depth
	     * @return Returns the overall number of pixels *data depth
	     */
	    inline uint32_t getNumberOfBytes() const {
		return getPixelSize()*getPixelCount();
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

	    inline void setImage(const std::vector<uint8_t> &image) {
		this->image = image;
	    }
	    inline void setImage(const char *data, uint32_t size) {
		if (size != this->image.size())
		{
		    std::cerr << "Frame: "
		              << __FUNCTION__ << " (" << __FILE__ << ", line "
		              << __LINE__ << "): " << "image size mismatch in setImage() ("
		              << size << " != " << this->image.size() << ")"
		              << std::endl;
		    return;
		}

		memcpy(&this->image[0], data, size);
	    }

	    inline uint8_t *getImagePtr() {
		return static_cast<uint8_t *>(&this->image[0]);
	    }
	    inline const uint8_t *getImageConstPtr() const {
		return static_cast<const uint8_t *>(&this->image[0]);
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
	    
	    ConstColumnIterator getColumnBegin(uint16_t column)const
	    {
	      if(column >= getWidth())
	      {
		 std::cerr << "Frame: "
		              << __FUNCTION__ << " (" << __FILE__ << ", line "
		              << __LINE__ << "): " << "out of index!"
		              << std::endl;
		return ConstColumnIterator();
	      }
	      const uint8_t *pdata = getImageConstPtr()+column*getPixelSize();
	      return ConstColumnIterator(getRowSize(),pdata,pdata+getRowSize()*(getHeight()-1));
	    }
	    
	    ConstColumnIterator end()const
	    {
	      static ConstColumnIterator iter;
	      return iter;
	    }
	    
	    template <typename Tp> Tp& at(unsigned int column,unsigned int row)
		{
	    	if(column >= size.width || row >= size.height )
	    		throw std::runtime_error("out of index");
	    	return *((Tp*)(getImagePtr()+row*getRowSize()+column*getPixelSize()));
		}

	    //check if opencv is present
	    #if defined( __OPENCV_CV_H__) ||defined (__OPENCV_CV_HPP__) || defined(_CV_H_) || defined(_CV_HPP_) || defined(__OPENCV_ALL_HPP__) ||defined(__OPENCV_OLD_CV_H__)
            int getOpenCvType()const
            {
		int itype = 0;
		switch (getChannelCount())
		{
		case 1:
		    switch (getPixelSize())
		    {
		    case 1:
		        itype = CV_8UC1;
		        break;
		    case 2:
		        itype = CV_16UC1;
		        break;
		    default:
		        throw "Unknown format. Can not convert Frame "
		        "to cv::Mat.";
		    }
		    break;
		case 3:
		    switch (getPixelSize())
		    {
		    case 3:
		        itype = CV_8UC3;
		        break;
		    case 6:
		        itype = CV_16UC3;
		        break;
		        throw "Unknown format. Can not convert Frame "
		        "to cv::Mat.";
		    }
		    break;
		    throw "Unknown format. Can not convert Frame "
		    "to cv::Mat.";
		}
                return itype;
            }

	    inline cv::Mat convertToCvMat()
	    {
		return cv::Mat(size.height,size.width, getOpenCvType(), getImagePtr());
	    }
	    inline const cv::Mat convertToCvMat()const
	    {
		return cv::Mat(size.height,size.width, getOpenCvType(), (void*)getImageConstPtr());
	    }
	    #else
	      #define convertToCvMat If_you_want_to_use_convertToCvMat_include_opencv_2_first
	    #endif
	 #endif

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

	    /** The number of effective bits per pixel. The number
	     * of actual bits per pixels is always a multiple of
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
