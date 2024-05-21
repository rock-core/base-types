#ifndef WORKSPACE_BASE_TYPES_SRC_LOGMESSAGE_HPP_
#define WORKSPACE_BASE_TYPES_SRC_LOGMESSAGE_HPP_

#include <string>
#include <base/Time.hpp>

namespace base {

/**
 * @brief Log message type to be able to e.g. write timestamped messages/markers into log files
 * 
 */
class LogMessage {
 public:
    base::Time time;
    std::string message;
};

}  // namespace base

#endif  // WORKSPACE_BASE_TYPES_SRC_LOGMESSAGE_HPP_

