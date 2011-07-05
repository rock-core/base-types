/*
 * @file Logger.cpp
 * @author Thomas Roehr, thomas.roehr@rock.de
 *
 */


#include <stdio.h>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include "logging.h"
#include "terminal_colors.h"

namespace base {
namespace logging { 

Logger::Logger() : mStream(stderr), mPriorityNames(10), mLogFormatNames(3)
{
    mPriorityNames[INFO] = "INFO";
    mPriorityNames[DEBUG] = "DEBUG";
    mPriorityNames[WARN] = "WARN";
    mPriorityNames[ERROR] = "ERROR";
    mPriorityNames[FATAL] = "FATAL";
    mPriorityNames[UNKNOWN] = "UNKNOWN";

    mLogFormatNames[DEFAULT] = "DEFAULT";
    mLogFormatNames[MULTILINE] = "MULTILINE";
    mLogFormatNames[SHORT] = "SHORT";
    mLogFormat = getLogFormatFromEnv();

    mPriority = getLogLevelFromEnv();

    if (getLogColorFromEnv())
    {
    	mpLogColor[DEBUG] =  COLOR_BIG;
    	mpLogColor[INFO] = COLOR_FG_WHITE;
    	mpLogColor[WARN] = COLOR_FG_LIGHTYELLOW;
    	mpLogColor[ERROR] = COLOR_FG_DARKRED;
    	mpLogColor[FATAL] = COLOR_BG_DARKRED;
    	mpLogColor[UNKNOWN] = COLOR_NORMAL;
    	mpColorEnd = COLOR_NORMAL;

    } else {
        for (int i = 0;i < ENDPRIORITIES;i++)
        {
        	mpLogColor[i] = "";
        }
        mpColorEnd = "";
    }

    // Per default enable ERROR logging
    if(mPriority == UNKNOWN)
        mPriority = ERROR;
}

Logger::~Logger()
{
}

void Logger::configure(Priority priority, FILE* outputStream)
{
    Priority envPriority = getLogLevelFromEnv();
    // Only limit to higher (close to FATAL) priorities
    if(envPriority < priority && envPriority != UNKNOWN)
        mPriority = envPriority;
    else
        mPriority = priority;

    if(outputStream)
        mStream = outputStream;
}

Priority Logger::getLogLevelFromEnv()
{
    char* loglevel = getenv("BASE_LOG_LEVEL");
    if(!loglevel)
        return UNKNOWN;

    std::string priority(loglevel);
    std::transform(priority.begin(), priority.end(),priority.begin(), (int(*)(int)) std::toupper);
    
    int index = 0;
    std::vector<std::string>::iterator it = mPriorityNames.begin();
    for(;it != mPriorityNames.end(); it++)
    {
        if(*it != priority)
        {
            index++;
        } else {
            return (Priority) index; 
        }
    }

    return UNKNOWN;

}

bool Logger::getLogColorFromEnv()
{
    char* color = getenv("BASE_LOG_COLOR");
    if(color)
        return true;

    return false;
}


LogFormat Logger::getLogFormatFromEnv()
{
    char* logtype = getenv("BASE_LOG_FORMAT");
    if(!logtype)
        return DEFAULT;

    std::string logtype_str(logtype);
    std::transform(logtype_str.begin(), logtype_str.end(),logtype_str.begin(), (int(*)(int)) std::toupper);

    std::vector<std::string>::iterator it = mLogFormatNames.begin();
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


void Logger::log(Priority priority, const char* function, const char* file, int line, const char* format, ...)
{
    if(priority <= mPriority)
    {
        int n;	
        char buffer[1024];
        va_list arguments;

        va_start(arguments, format);
        n = vsnprintf(buffer, sizeof(buffer), format, arguments);
        va_end(arguments);

        time_t now;
        time(&now);
        struct tm* current = localtime(&now);
        char currentTime[25];

        struct timeval tv;
        gettimeofday(&tv,0);
        int milliSecs = tv.tv_usec/1000;

        strftime(currentTime, 25, "%Y%m%d-%H:%M:%S", current);

        //Todo: optional log pattern, e.g. %t(ime) %p(rio) %f(unc) %m(sg) %F(ile) %L(ine)
        //could be optimized to a jumplist, so no performance issue
        switch (mLogFormat)
        {
            case DEFAULT:
                fprintf(mStream, "[%s:%03d] %s[%5s] - %s%s (%s:%d - %s)\n", currentTime, milliSecs, mpLogColor[priority], mPriorityNames[priority].c_str(), buffer, mpColorEnd, file, line, function);
                break;
            case MULTILINE:
                fprintf(mStream, "[%s:%03d] in %s\n\t%s:%d\n\t%s[%5s] - %s%s \n", currentTime, milliSecs, function, file, line, mpLogColor[priority], mPriorityNames[priority].c_str(), buffer, mpColorEnd);
                break;
            case SHORT:
                fprintf(mStream, "%s[%5s] - %s%s\n", mpLogColor[priority], mPriorityNames[priority].c_str(), buffer, mpColorEnd);
                break;
       }
    }
}

} // end namespace logging
} // end namespace base

