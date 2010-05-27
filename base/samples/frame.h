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
#endif

#include "../time.h"


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
	#endif
	    uint16_t width;
	    uint16_t height;
	};

	enum frame_mode_t {
	    MODE_UNDEFINED = 0,
	    MODE_GRAYSCALE = 1,
	    MODE_RGB       = 2,
	    RAW_MODES = 128,
	    MODE_BAYER_RGGB = RAW_MODES + 0,
	    MODE_BAYER_GRBG = RAW_MODES + 1,
	    MODE_BAYER_BGGR = RAW_MODES + 2,
	    MODE_BAYER_GBRG = RAW_MODES + 3

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
	class ConstColumnIterator
	{
	  private:
	    uint32_t row_size;
	    const uint8_t* pdata;
	    const uint8_t* pend;
	  public:
	  ConstColumnIterator(uint32_t _row_size,const uint8_t* _pdata, const uint8_t* _pend)
	    :row_size(_row_size),pdata(_pdata),pend(_pend){}
	  ConstColumnIterator()  //end iterator
	  {row_size =0;pdata = NULL;pend = NULL;}
	  
	  ConstColumnIterator &operator++()
	  {
	    if(pdata >= pend)
	    {
	      pdata = NULL;
	      pend = NULL;
	      return *this;
	    }
	    pdata += row_size;
	    return *this;
	  }
	  bool operator==(const ConstColumnIterator &other)const{ return (pdata == other.pdata);}
	  const uint8_t* operator*()const{return pdata;}
	  bool operator==(uint8_t* p)const{return (pdata == p);}
	  bool operator!=(uint8_t* p)const{return !(pdata == p);}
	  bool operator!=(const ConstColumnIterator &other)const{return !(pdata == other.pdata);}
	  void operator=(const ConstColumnIterator &other)
	  {row_size = other.row_size;pdata = other.pdata;pend = other.pend;}
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
	    
	    Frame(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, bool hdr = false)
	    {
		init(width,height,depth,mode,hdr);
	    }
	    
	    //makes a copy of other
	    Frame(const Frame &other,bool bcopy = true)
	    {
		init(other,bcopy);
	    }
	    
	    //makes a copy of other
	    void init(const Frame &other,bool bcopy = true)
	    {
	       //hdr is copied by attributes = other.attributes;
	       //change size if the frame does not fit
	       if(other.getHeight() != getHeight() || other.getWidth() !=  getWidth() || other.getFrameMode() != getFrameMode())
		  init(other.getWidth(),other.getHeight(),other.getDataDepth(),other.getFrameMode(),false);
	       if(bcopy)
		  setImage(other.getImage());
	       attributes = other.attributes;
	       time = other.time;
	       received_time = other.received_time;
	       frame_status = other.frame_status;
	    }
	    
	    void init(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, bool hdr = false)
	    {
		this->frame_mode = mode;
		this->size = frame_size_t(width, height);
		setDataDepth(depth);
		image.resize(getPixelSize() * getPixelCount());
		reset();
		if(hdr)
		  setAttribute<bool>("hdr",true);
	    }

	    void reset()
	    {
		this->time = base::Time(0);
		if (this->image.size() > 0) {
		    memset(&this->image[0], 0, this->image.size());
		}
		setStatus(STATUS_EMPTY);
		attributes.clear();
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
		case MODE_BAYER_RGGB:
		    return 1;
		case MODE_BAYER_BGGR:
		    return 1;
		case MODE_BAYER_GBRG:
		    return 1;
		case MODE_BAYER_GRBG:
		    return 1;
		case MODE_GRAYSCALE:
		    return 1;
		case MODE_RGB:
		    return 3;
		default:
		    return 0;
		}
	    }

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
	    
	    ConstColumnIterator getColumnBegin(uint8_t column)const
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
	      return ConstColumnIterator(getRowSize(),pdata,pdata+getRowSize()*getHeight());
	    }
	    
	    ConstColumnIterator end()
	    {
	      static ConstColumnIterator iter;
	      return iter;
	    }
	    
	    //check if opencv is present
	    #if defined( __OPENCV_CV_H__) ||defined (__OPENCV_CV_HPP__) || defined(_CV_H_) || defined(_CV_HPP_)
	    inline cv::Mat convertToCvMat()
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
		return cv::Mat(size.height,size.width, itype, getImagePtr());
	    }
	    #elsef
	      #define convertToCvMat If_you_want_to_use_convertToCvMat_include_opencv_2_first
	    #endif
	 #endif

	    // The unix time at which the camFrame was captured
	    base::Time              time;

            // The unix time at which the camFrame was received
	    base::Time              received_time;

	    std::vector<uint8_t>    	image;
	    std::vector<frame_attrib_t> attributes;

	    // The image size [width, height]
	    frame_size_t            size;

	    // The number of effective bits per pixel. The number
	    // of actual bits per pixels is always a multiple of
	    // height (i.e. a 12-bit effective depth is represented
	    // using 16-bits per channels). The number of greyscale
	    // levels is 2^(this_number)
	    uint32_t                data_depth;
	    uint32_t                pixel_size;
	    uint32_t 		    row_size;

	    frame_mode_t            frame_mode;
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