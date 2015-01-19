#ifndef _BASE_LOGGING_H_
#define _BASE_LOGGING_H_

/*
 * @file Logging.hpp 
 *
 * @brief Basic logging functionality 
 * @details Logging can be enable by linking to base-lib and including <base/Logging.hpp>
 *
 * At compile time the following defines can be set: 
 * Setting the namespace to facilitate associating the debug with a library
 * BASE_LOG_NAMESPACE, e.g. 
 * in your CMakeLists.txt add_defitions(-DBASE_LOG_NAMESPACE=$PROJECT_NAME)
 * 
 * Allow only logs of a certain level or higher. This is for compile time, so any logs below
 * this level will not be compiled in.
 * BASE_LOG_<log-level>, e.g. BASE_LOG_FATAL
 *
 * To disable logging:
 * BASE_LOG_DISABLE
 * 
 * Existing log levels are: FATAL, ERROR, WARN, INFO, DEBUG
 *
 * At runtime the enviroment variable BASE_LOG_LEVEL can be set to any of the 
 * given log levels, e.g. export BASE_LOG_LEVEL="info" to show debug of 
 * INFO and higher log statements
 *
 * Setting of BASE_LOG_COLOR enables a color scheme for the log message, that 
 * is best viewed in a terminal with dark background color
 *
 */

#include <base/logging/logging_printf_style.h>
#include <base/logging/logging_iostream_style.h>
#endif // _BASE_LOGGING_H_
