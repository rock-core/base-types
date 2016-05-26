/*
 * @file logging_iostream_style.h
 * @author Jan Vogelgesang, jan.vogelgesang@dfki.de
 *
 * @brief Wrapper for logging.h, adds iostream-style logging
 *
 * @details adds an iostream-style interface to logging.h, appends trailing '_S' for the
 *          iostream interface. Example:
 *
 *          LOG_WARN_S << "some warning message";
 *
 *          no trailing endl is required (similar to logging_printf_style.h).
 *          Configuration is done through LOG_CONFIGURE from logging_printf_style.h.
 *
 *          based on an idea from Petru Marginean, presented at http://drdobbs.com/cpp/201804215
 */

#ifndef _BASE_LOGGING_IOSTREAM_STYLE_H_
#define _BASE_LOGGING_IOSTREAM_STYLE_H_

#include <base/logging/logging_printf_style.h>
#include <sstream>

// To allow for the streaming syntax, relying on 'dead code elimination'
// of the compiler, i.e. given the if prio > BASE_LOG_PRIORITY then the else branch
// is never reachable. Compilers with 'dead code elimination'
// will remove the else branch completely (tested on gcc 4.4 with -o0).
// Using __PRETTY_FUNCTION__ when using gcc otherwise __func__ to show current function
#ifdef __GNUC__
#define LOG_STREAM(PRIO) if(PRIO > BASE_LOG_PRIORITY) ; else base::logging::LogStream().get(PRIO,__PRETTY_FUNCTION__, __FILE__, __LINE__, __STRINGIFY(BASE_LOG_NAMESPACE))
#else
#define LOG_STREAM(PRIO) if(PRIO > BASE_LOG_PRIORITY) ; else base::logging::LogStream().get(PRIO,__func__, __FILE__, __LINE__,  __STRINGIFY(BASE_LOG_NAMESPACE))
#endif


// MACROS for general usage of streaming logging
// Example usage:
// LOG_WARN_S << "this is a message on level warn";
//
// NOTE:'std::endl' is inserted automatically at the end of the statement 
//
#define LOG_FATAL_S LOG_STREAM(base::logging::FATAL)
#define LOG_ERROR_S  LOG_STREAM(base::logging::ERROR)
#define LOG_WARN_S  LOG_STREAM(base::logging::WARN)
#define LOG_INFO_S  LOG_STREAM(base::logging::INFO)
#define LOG_DEBUG_S  LOG_STREAM(base::logging::DEBUG)

namespace base {

namespace logging {

class LogStream
{
public:
	LogStream() {};
	virtual ~LogStream()
	{
	    // forward it to class Logger
	    Logger::getInstance()->logBuffer(mPrio,mpFuncName,mpFileName,mLineNumber,mNamespace,os.str().c_str());
	}


	std::ostringstream& get(Priority prio,const char* pFuncName, const char* pFileName, int lineNumber, const char* name_space)
	{
	    mPrio = prio;
	    mpFuncName = pFuncName;
	    mpFileName = pFileName;
	    mLineNumber = lineNumber;
	    mNamespace = name_space;
	   return os;
	}

protected:
   std::ostringstream os;
private:
   LogStream(const LogStream&);
   LogStream& operator =(const LogStream&);
private:
   Priority mPrio;
   const char* mpFuncName;
   const char* mpFileName;
   int mLineNumber;
   const char* mNamespace;
};


} //namespace logging
} //namespace base

#endif // _BASE_LOGGING_IOSTREAM_STYLE_H_
