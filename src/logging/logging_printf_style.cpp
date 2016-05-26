/*
 * @file logging_printf_style.cpp
 * @author Thomas Roehr, thomas.roehr@rock.de
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include "terminal_colors.h"
#include "logging_printf_style.h"

namespace base {
namespace logging { 

Logger::Logger() : mStream(stderr), mPriorityNames(ENDPRIORITIES), mLogFormatNames(ENDLOGFORMATS)
{
    mPriorityNames[INFO_P] = "INFO";
    mPriorityNames[DEBUG_P] = "DEBUG";
    mPriorityNames[WARN_P] = "WARN";
    mPriorityNames[ERROR_P] = "ERROR";
    mPriorityNames[FATAL_P] = "FATAL";
    mPriorityNames[UNKNOWN_P] = "UNKNOWN";

    mLogFormatNames[DEFAULT] = "DEFAULT";
    mLogFormatNames[MULTILINE] = "MULTILINE";
    mLogFormatNames[SHORT] = "SHORT";
    mLogFormat = getLogFormatFromEnv();

    mPriority = getLogLevelFromEnv();

    if (getLogColorFromEnv())
    {
    	mpLogColor[DEBUG_P] =  COLOR_BIG;
    	mpLogColor[INFO_P] = COLOR_FG_WHITE;
    	mpLogColor[WARN_P] = COLOR_FG_LIGHTYELLOW;
    	mpLogColor[ERROR_P] = COLOR_FG_DARKRED;
    	mpLogColor[FATAL_P] = COLOR_BG_DARKRED;
    	mpLogColor[UNKNOWN_P] = COLOR_NORMAL;
    	mpColorEnd = COLOR_NORMAL;

    } else {
        for (int i = 0;i < ENDPRIORITIES;i++)
        {
        	mpLogColor[i] = "";
        }
        mpColorEnd = "";
    }

    // Per default enable ERROR logging
    if(mPriority == UNKNOWN_P)
        mPriority = ERROR_P;
}

Logger::~Logger()
{
}

void Logger::configure(Priority priority, FILE* outputStream)
{
    Priority envPriority = getLogLevelFromEnv();
    // Only limit to higher (close to FATAL) priorities
    if(envPriority < priority && envPriority != UNKNOWN_P)
        mPriority = envPriority;
    else
        mPriority = priority;

    if(outputStream)
        mStream = outputStream;
}

Priority Logger::getLogLevelFromEnv() const
{
    const char* loglevel = getenv("BASE_LOG_LEVEL");
    if(!loglevel)
        return UNKNOWN_P;

    std::string priority(loglevel);
    std::transform(priority.begin(), priority.end(),priority.begin(), (int(*)(int)) std::toupper);
    
    int index = 0;
    std::vector<std::string>::const_iterator it = mPriorityNames.begin();
    for(;it != mPriorityNames.end(); it++)
    {
        if(*it != priority)
        {
            index++;
        } else {
            return (Priority) index; 
        }
    }

    return UNKNOWN_P;
}

bool Logger::getLogColorFromEnv() const
{
    const char* color = getenv("BASE_LOG_COLOR");
    if(color)
        return true;

    return false;
}


LogFormat Logger::getLogFormatFromEnv() const
{
    const char* logtype = getenv("BASE_LOG_FORMAT");
    if(!logtype)
        return DEFAULT;

    std::string logtype_str(logtype);
    std::transform(logtype_str.begin(), logtype_str.end(),logtype_str.begin(), (int(*)(int)) std::toupper);

    std::vector<std::string>::const_iterator it = mLogFormatNames.begin();
    int index = 0;
    for(;it != mLogFormatNames.end(); it++)
    {
        if(*it != logtype_str)
        {
            index++;
        } else {
            return (LogFormat) index;
        }
    }

    return DEFAULT;
}


void Logger::log(Priority priority, const char* function, const char* file, int line, const char* name_space, const char* format, ...) const
{
    if(priority <= mPriority)
    {
        char buffer[1024];
        va_list arguments;

        va_start(arguments, format);
        vsnprintf(buffer, sizeof(buffer), format, arguments);
        va_end(arguments);

        logBuffer(priority,function,file,line,name_space,buffer);
    }
}

void Logger::logBuffer(Priority priority, const char* function, const char* file, int line, const char* name_space, const char* buffer) const
{
    if(priority <= mPriority)
    {
        time_t now;
        time(&now);
        struct tm* current = localtime(&now);
        char currentTime[25];

        struct timeval tv;
        gettimeofday(&tv,0);
        int milliSecs = tv.tv_usec/1000;

        strftime(currentTime, sizeof(currentTime), "%Y%m%d-%H:%M:%S", current);

        //Todo: optional log pattern, e.g. %t(ime) %p(rio) %f(unc) %m(sg) %F(ile) %L(ine)
        //could be optimized to a jumplist, so no performance issue
        switch (mLogFormat)
        {
            case ENDLOGFORMATS:
            case DEFAULT:
                fprintf(mStream, "[%s:%03d] %s[%5s] - %s::%s%s (%s:%d - %s)\n", currentTime, milliSecs, mpLogColor[priority], mPriorityNames[priority].c_str(), name_space, buffer, mpColorEnd, file, line, function);
                break;
            case MULTILINE:
                fprintf(mStream, "[%s:%03d] in %s\n\t%s:%d\n\t%s[%5s] - %s::%s%s \n", currentTime, milliSecs, function, file, line, mpLogColor[priority], mPriorityNames[priority].c_str(), name_space, buffer, mpColorEnd);
                break;
            case SHORT:
                fprintf(mStream, "%s[%5s] - %s::%s%s\n", mpLogColor[priority], mPriorityNames[priority].c_str(), name_space, buffer, mpColorEnd);
                break;
       }
        fflush(mStream);
    }
}

} // end namespace logging
} // end namespace base

