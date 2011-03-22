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
	    RAW_MODES = 128,
	    MODE_BAYER = RAW_MODES + 0,
	    MODE_BAYER_RGGB = RAW_MODES + 1,
	    MODE_BAYER_GRBG = RAW_MODES + 2,
	    MODE_BAYER_BGGR = RAW_MODES + 3,
	    MODE_BAYER_GBRG = RAW_MODES + 4
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
	    
	    Frame(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, uint8_t const val = 0, bool hdr = false)
	    {
		init(width,height,depth,mode,val,hdr);
	    }
	    
	    //makes a copy of other
	    Frame(const Frame &other,bool bcopy = true)
	    {
		init(other,bcopy);
	    }
	    
	    //copies all attributes which are independant from size and mode
	    void copyImageIndependantAttributes(const Frame &other)
	    {
	       attributes = other.attributes;
	       time = other.time;
	       received_time = other.received_time;
	       frame_status = other.frame_status;
	    }
	    
	    //makes a copy of other
	    void init(const Frame &other,bool bcopy = true)
	    {
	       //hdr is copied by attributes = other.attributes;
	       //change size if the frame does not fit
	       if(other.getHeight() != getHeight() || other.getWidth() !=  getWidth() || other.getFrameMode() != getFrameMode())
		  init(other.getWidth(),other.getHeight(),other.getDataDepth(),other.getFrameMode(), -1, false);
	       if(bcopy)
		  setImage(other.getImage());
	       copyImageIndependantAttributes(other);
	    }
	    
	    void init(uint16_t width, uint16_t height, uint8_t depth, frame_mode_t mode, int const val = 0, bool hdr = false)
	    {

		this->frame_mode = mode;
		this->size = frame_size_t(width, height);
		setDataDepth(depth);
		image.resize(getPixelSize() * getPixelCount());
		reset(val);
		if(hdr)
		  setAttribute<bool>("hdr",true);
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
		case MODE_BAYER:
		    return 1;
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
		case MODE_UYVY:
		    return 1;
		case MODE_RGB:
		    return 3;
		default:
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
	    #if defined( __OPENCV_CV_H__) ||defined (__OPENCV_CV_HPP__) || defined(_CV_H_) || defined(_CV_HPP_) || defined(__OPENCV_ALL_HPP__)
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
	    #else
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
