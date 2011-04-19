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

namespace base {
namespace logging { 

Logger::Logger() : mStream(stderr), mPriorityNames(10)

{
    mPriorityNames[INFO] = "INFO";
    mPriorityNames[DEBUG] = "DEBUG";
    mPriorityNames[WARN] = "WARN";
    mPriorityNames[ERROR] = "ERROR";
    mPriorityNames[FATAL] = "FATAL";
    mPriorityNames[UNKNOWN] = "UNKNOWN";

    mPriority = getLogLevelFromEnv();

    // Per default enable ERROR logging
    if(mPriority == UNKNOWN)
        mPriority == ERROR;
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
    
    std::vector<std::string>::iterator it = mPriorityNames.begin();
    int index = 0;
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

void Logger::log(Priority priority, const char* file, int line, const char* format, ...)
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
             
            fprintf(mStream, "[%s:%03d][%s] - %s (%s:%d)\n", currentTime, milliSecs,  mPriorityNames[priority].c_str(), buffer, file, line);
        }
}

} // end namespace logging
} // end namespace base

