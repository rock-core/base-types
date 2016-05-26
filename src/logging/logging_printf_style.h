/*
 * @file logging_printf_style.h
 *
 * @brief printf style logging 
 * @details defines printf like logging functions 
 *
 * To write a log message, use LOG_<log-level> in the code. E.g.
 *
 * LOG_DEBUG("Reached this point")
 * LOG_INFO("Value is %i", value)
 * 
 * are valid statements.
 * if BASE_LONG_NAMES is defined, the statements are prefixed with BASE_ e.g.:
 *
 * BASE_LOG_FATAL("Cannot reach device.")
 */

#ifndef _BASE_LOGGING_PRINTF_STYLE_H_
#define _BASE_LOGGING_PRINTF_STYLE_H_

// Need to map to logging::Priority order
// Allowing to set a log level via CFLAGS - method call will not be compiled into the system
// Setting the environment variable BASE_LOG_LEVEL does only have any effect if the compiled
// level is equal or higher (closer to FATAL) than the one request
//
#if defined(BASE_LOG_DISABLE)
#define BASE_LOG_PRIORITY 0
#elif defined(BASE_LOG_FATAL)
#define BASE_LOG_PRIORITY 1
#undef BASE_LOG_FATAL
#elif defined(BASE_LOG_ERROR)
#define BASE_LOG_PRIORITY 2
#undef BASE_LOG_ERROR
#elif defined(BASE_LOG_WARN)
#define BASE_LOG_PRIORITY 3
#undef BASE_LOG_WARN
#elif defined(BASE_LOG_INFO)
#define BASE_LOG_PRIORITY 4
#undef BASE_LOG_INFO
#elif defined(BASE_LOG_DEBUG)
#define BASE_LOG_PRIORITY 5
#undef BASE_LOG_DEBUG
#endif

#ifndef BASE_LOG_PRIORITY
// Default logging priority that is compiled in, i.e. all log levels 
// will be accessible at runtime
#define BASE_LOG_PRIORITY 6
#endif

#ifdef BASE_LONG_NAMES
// Empty definition of debug statement
#define BASE_LOG_DEBUG(FORMAT, ARGS...)
#define BASE_LOG_INFO(FORMAT, ARGS...)
#define BASE_LOG_WARN(FORMAT, ARGS...)
#define BASE_LOG_ERROR(FORMAT, ARGS...)
#define BASE_LOG_FATAL(FORMAT, ARGS...)
#define BASE_LOG_CONFIGURE(PRIO, STREAM)
#else
#define LOG_DEBUG(FORMAT, ARGS...)
#define LOG_INFO(FORMAT, ARGS...)
#define LOG_WARN(FORMAT, ARGS...)
#define LOG_ERROR(FORMAT, ARGS...)
#define LOG_FATAL(FORMAT, ARGS...)
#define LOG_CONFIGURE(PRIO, STREAM)
#endif // BASE_LONG_NAMES

#ifndef BASE_LOG_NAMESPACE
#define BASE_LOG_NAMESPACE ""
#endif // BASE_LOG_NAMESPACE
// The debug flag BASE_LOG_NAMESPACE needs to be converted to a string
// Stringify element
#define __STRINGIFY_(X) #X
// expand x before being stringified
#define __STRINGIFY(X) __STRINGIFY_(X)

// Depending on the globally set log level insert log statements by preprocessor
//
// The namespace represents the library name and should be set via definitions, e.g. in
// your CMakeLists.txt -DBASE_LOG_NAMESPACE=yournamespace
//
// Using __PRETTY_FUNCTION__ when using gcc otherwise __func__ to show current function
#ifdef __GNUC__
#define __LOG(PRIO, FORMAT, ARGS ...) { ::base::logging::Logger::getInstance()->log(::base::logging::PRIO,__PRETTY_FUNCTION__, __FILE__, __LINE__, __STRINGIFY(BASE_LOG_NAMESPACE), FORMAT, ## ARGS); }
#else
#define __LOG(PRIO, FORMAT, ARGS ...) { ::base::logging::Logger::getInstance()->log(::base::logging::PRIO,__func__, __FILE__, __LINE__, __STRINGIFY(BASE_LOG_NAMESPACE),  FORMAT, ## ARGS); }
#endif

#ifdef BASE_LONG_NAMES

#if BASE_LOG_PRIORITY >= 1 
#undef BASE_LOG_FATAL
#define BASE_LOG_FATAL(FORMAT, ARGS...) __LOG(FATAL_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 2
#undef BASE_LOG_ERROR
#define BASE_LOG_ERROR(FORMAT, ARGS...) __LOG(ERROR_P, FORMAT, ## ARGS)
#endif
 
#if BASE_LOG_PRIORITY >= 3
#undef BASE_LOG_WARN
#define BASE_LOG_WARN(FORMAT, ARGS...) __LOG(WARN_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 4 
#undef BASE_LOG_INFO
#define BASE_LOG_INFO(FORMAT, ARGS...) __LOG(INFO_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 5
#undef BASE_LOG_DEBUG
#define BASE_LOG_DEBUG(FORMAT, ARGS...) __LOG(DEBUG_P, FORMAT, ## ARGS)  
#endif

#undef BASE_LOG_CONFIGURE
#define BASE_LOG_CONFIGURE(PRIO,STREAM) { ::base::logging::Logger::getInstance()->configure(::base::logging::PRIO, STREAM); }

#else // #ifdef BASE_LONG_NAMES

// If there should be conflicts with other libraries, switch to long names
// Other wise short names will be available as only
#undef LOG_CONFIGURE
#define LOG_CONFIGURE(PRIO,STREAM) { ::base::logging::Logger::getInstance()->configure(::base::logging::PRIO, STREAM); }

#if BASE_LOG_PRIORITY >= 1 
#undef LOG_FATAL
#define LOG_FATAL(FORMAT, ARGS...) __LOG(FATAL_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 2
#undef LOG_ERROR
#define LOG_ERROR(FORMAT, ARGS...) __LOG(ERROR_P, FORMAT, ## ARGS)
#endif
 
#if BASE_LOG_PRIORITY >= 3
#undef LOG_WARN
#define LOG_WARN(FORMAT, ARGS...) __LOG(WARN_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 4 
#undef LOG_INFO
#define LOG_INFO(FORMAT, ARGS...) __LOG(INFO_P, FORMAT, ## ARGS)
#endif

#if BASE_LOG_PRIORITY >= 5
#undef LOG_DEBUG
#define LOG_DEBUG(FORMAT, ARGS...) __LOG(DEBUG_P, FORMAT, ## ARGS)  
#endif

#endif // BASE_LONG_NAMES




#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <base/Singleton.hpp>

namespace base {

namespace logging {

/**
* To avoid clashes on WIN32 platform we use a _P suffix for the priorities
* Still allowing to use without prefix on other systems
*/
#ifdef WIN32
enum Priority	{ UNKNOWN_P = 0, FATAL_P , ERROR_P, WARN_P, INFO_P, DEBUG_P, ENDPRIORITIES };
#else
enum Priority	{ UNKNOWN = 0, UNKNOWN_P = 0, FATAL = 1, FATAL_P =1, ERROR = 2, ERROR_P = 2, WARN = 3, WARN_P = 3, INFO = 4, INFO_P = 4, DEBUG = 5, DEBUG_P = 5, ENDPRIORITIES };
#endif
 

enum LogFormat	{ DEFAULT = 0, MULTILINE, SHORT, ENDLOGFORMATS};

/**
 * @class Logger
 * @brief Logger is a logger that allows priority based logging
 * with minimal impact on performance and minimal configuration
 * requirements
 * 
 * Use the environment variable BASE_LOG_LEVEL to define the requested logging
 * level, e.g.:
  \code{.sh}
  export BASE_LOG_LEVEL="WARN"
  \endcode
 *
 * 
 * A library designer can decide using the BASE_LOG_xxx flag at compile time which log level
 * should be available. 
 * 
 * So switch the output stream (for example logging into a file) the following
 * can be used:
 *
   \verbatim
   BASE_LOG_CONFIGURE(priority,ostream)
   \endverbatim
 *
 */
class Logger : public Singleton<Logger>
{
    friend class Singleton<Logger>;

protected:
	/**
	 * Construct the logger
	 */
	Logger();
public:

	virtual ~Logger();

        /** 
        * Configure logger - this is for the library developer so he can set a
        * maximum log level, which cannot be further limited to higher log
        * priorities via setting BASE_LOG_LEVEL.
        *
        * If no previous configuration is given, no output logging will be done
        */
        void configure(Priority priority, FILE* outputStream);
	
	/**
	* Logs a message with a given priority, can be used with printf style format
	* @param priority priority level
        * @param ns namespace to be used
        * @param filename Filename
        * @param line Linenumber
	* @param format printf like format string
	* @param ... variable argument list
	*/
	void log(Priority priority, const char* function, const char* filename, int line, const char* name_space, const char* format, ...) const;

	/**
	 * used by log(...)
	 * logs the text contained in buffer.
	 */
	void logBuffer(Priority priority, const char* function, const char* file, int line, const char* name_space, const char* buffer) const;

private:
        /**
        * Retrieve the log level from the environment variable BASE_LOG_LEVEL.
        * Allows to influence verbosity of logging.
        */
        Priority getLogLevelFromEnv() const;

        /**
        * Query environment for the existence of BASE_LOG_COLOR variable. If
        * set, the output will be coloured.
        */
        bool getLogColorFromEnv() const;

        /** 
        * Retrieve log level from the environment variable BASE_LOG_FORMAT.
        * Allows to set single- or multi line output.
        */
        LogFormat getLogFormatFromEnv() const;

        FILE* mStream;
        std::vector<std::string> mPriorityNames;
        Priority mPriority;

        const char* mpLogColor[ENDPRIORITIES];
        const char* mpColorEnd;

        std::vector<std::string> mLogFormatNames;
        LogFormat mLogFormat;
};

} // end namespace
} // end namespace

#endif /* _BASE_LOGGING_PRINTF_STYLE_H_ */

