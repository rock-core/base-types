/*
 * @file logging.h
 * @author Thomas Roehr, thomas.roehr@dfki.de
 *
 * @brief Plain logging class
 */

#ifndef _BASE_LOGGING_H_
#define _BASE_LOGGING_H_

// Need to map to logging::Priority order
// ALlowing to set a log level via CFLAGS - method call will not be compiled into the system
// and setting the enviroment variable BASE_LOG_LEVEL does not have any effect
#if defined(BASE_LOG_FATAL)
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
#define BASE_LOG_PRIORITY 6
#endif

#define BASE_LOG_DEBUG(FORMAT, ARGS...)
#define BASE_LOG_INFO(FORMAT, ARGS...)
#define BASE_LOG_WARN(FORMAT, ARGS...)
#define BASE_LOG_ERROR(FORMAT, ARGS...)
#define BASE_LOG_FATAL(FORMAT, ARGS...)
#define BASE_LOG_INIT(NS, PRIO, STREAM)

#ifndef Release

#if BASE_LOG_PRIORITY >= 1 
#undef BASE_LOG_FATAL
#define BASE_LOG_FATAL(FORMAT, ARGS...) { using namespace logging; Logger::getInstance()->log(FATAL,__FILE__, __LINE__, FORMAT, ## ARGS); }
#endif

#if BASE_LOG_PRIORITY >= 2
#undef BASE_LOG_ERROR
#define BASE_LOG_ERROR(FORMAT, ARGS...) { using namespace logging; Logger::getInstance()->log(ERROR, __FILE__, __LINE__, FORMAT, ## ARGS); }
#endif
 
#if BASE_LOG_PRIORITY >= 3
#undef BASE_LOG_WARN
#define BASE_LOG_WARN(FORMAT, ARGS...) { using namespace logging; Logger::getInstance()->log(WARN,__FILE__, __LINE__, FORMAT, ## ARGS); }
#endif

#if BASE_LOG_PRIORITY >= 4 
#undef BASE_LOG_INFO
#define BASE_LOG_INFO(FORMAT, ARGS...) { using namespace logging; Logger::getInstance()->log(INFO, __FILE__, __LINE__, FORMAT, ## ARGS); }
#endif

#if BASE_LOG_PRIORITY >= 5
#undef BASE_LOG_DEBUG
#define BASE_LOG_DEBUG(FORMAT, ARGS...) { using namespace logging; Logger::getInstance()->log(DEBUG,__FILE__, __LINE__, FORMAT, ## ARGS); }
#endif

#undef BASE_LOG_INIT
#define BASE_LOG_INIT(NS, PRIO, STREAM) { using namespace logging; Logger::getInstance()->configure(NS, PRIO, STREAM); }

#endif

#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <base/singleton.h>

using namespace std;

namespace logging {

	enum Priority	{ UNKNOWN = 0, FATAL , ERROR, WARN, INFO, DEBUG  };

	/**
	 * @class Logger
	 * @brief Logger is a logger that allows priority based logging
         * with minimal impact on performance and minimal configuration
         * requirements
         * 
         * The logger will be only active in an application compiled 
         * with DEBUG flags 
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

                /** 
                * Configure logger - this is for the library developer so he can set a maximum log
                * level, which cannot be further limited to higher log priorities via setting BASE_LOG_LEVEL
                * If no previous configuration is given, no output logging will be done
                */
                void configure(const std::string& _namespace, Priority priority, FILE* outputStream);

		virtual ~Logger();
		
		/**
 		* Logs a message with a given priority, can be used with printf style format
 		* @param priority priority level
 		* @param format string
 		* @param ... variable argument list
 		*/
		void log(Priority priority, const char* file, int line, const char* format, ...);

	private:
                /**
                * Retrieve the log level from the enviroment
                */
                Priority getLogLevelFromEnv();

		std::string mNamespace;
                FILE* mStream;
                std::vector<std::string> mPriorityNames;
                Priority mPriority;

	};

} // end namespace

#endif /* _BASE_LOGGER_H_ */

