#ifndef LOG_MESSAGE_HPP
#define LOG_MESSAGE_HPP

#include <string>
#include <base/Time.hpp>

namespace base
{

    /**
     * @brief Log message type to be able to e.g. write timestamped messages/markers into log files
     * 
     */
    class LogMessage
    {
        public:
            base::Time timestamp;
            std::string message;
    };

}

#endif

