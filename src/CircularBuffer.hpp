// Until Boost 1.62.0, the ABI boost's circular_buffer.hpp would change between
// release and debug modes (plus, it seems, the debug version is not thread)
// safe. Rock shipped this header to make sure the ABI stayed the same
//
// 1.62.0 shipped in 2016, we don't have to support this anymore

#include <boost/version.hpp>
#if BOOST_VERSION < 106200
#  error "the reason why base/CircularBuffer.hpp had been created is not valid since 1.62.0, we stopped supporting our modified version of the header and therefore do not support Boost < 1.62.0"
#else
#  warning "the reason why base/CircularBuffer.hpp had been created is not valid anymore, use boost/circular_buffer.hpp directly instead"
#  include <boost/circular_buffer.hpp>
#endif
