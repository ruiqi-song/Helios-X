/******************************************************************************
 * Copyright 2022 The Helios-X Authors. All Rights Reserved.
 * Author: Ricky Song
 * Time: 2022-2-12
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cassert>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

#include "logger_base.h"


using GSeverity = LoggerBase::Severity;
using TRTSeverity = nvinfer1::ILogger::Severity;

class LogStreamBuffer : public std::stringbuf
{
public:
    LogStreamBuffer(std::ostream& stream, const std::string& prefix, bool shouldLog)
            : mOutput(stream)
            , mPrefix(prefix)
            , mShouldLog(shouldLog)
    {
    }

    LogStreamBuffer(LogStreamBuffer&& other)
            : mOutput(other.mOutput)
    {
    }

    ~LogStreamBuffer()
    {
        // std::streambuf::pbase() gives a pointer to the beginning of the buffered part of the output sequence
        // std::streambuf::pptr() gives a pointer to the current position of the output sequence
        // if the pointer to the beginning is not equal to the pointer to the current position,
        // call putOutput() to log the output to the stream
        if (pbase() != pptr())
        {
            putOutput();
        }
    }

    // synchronizes the stream buffer and returns 0 on success
    // synchronizing the stream buffer consists of inserting the buffer contents into the stream,
    // resetting the buffer and flushing the stream
    virtual int sync()
    {
        putOutput();
        return 0;
    }





    void putOutput()
    {
        if (mShouldLog)
        {
            // prepend timestamp
            std::time_t timestamp = std::time(nullptr);
            tm* tm_local = std::localtime(&timestamp);
            std::cout << "[";
            std::cout << std::setw(2) << std::setfill('0') << 1 + tm_local->tm_mon << "/";
            std::cout << std::setw(2) << std::setfill('0') << tm_local->tm_mday << "/";
            std::cout << std::setw(4) << std::setfill('0') << 1900 + tm_local->tm_year << "-";
            std::cout << std::setw(2) << std::setfill('0') << tm_local->tm_hour << ":";
            std::cout << std::setw(2) << std::setfill('0') << tm_local->tm_min << ":";
            std::cout << std::setw(2) << std::setfill('0') << tm_local->tm_sec << "] ";
            // std::stringbuf::str() gets the string contents of the buffer
            // insert the buffer contents pre-appended by the appropriate prefix into the stream
            mOutput << mPrefix << str();
            // set the buffer to empty
            str("");
            // flush the stream
            mOutput.flush();
        }
    }

    void setShouldLog(bool shouldLog)
    {
        mShouldLog = shouldLog;
    }

private:
    std::ostream& mOutput;
    std::string mPrefix;
    bool mShouldLog;
};

//!
//! \class LogStreamConsumerBase
//! \brief Convenience object used to initialize LogStreamConsumerBuffer before std::ostream in LogStreamConsumer
//!
class LogStreamBase
{
public:
    LogStreamBase(std::ostream& stream, const std::string& prefix, bool shouldLog)
            : mBuffer(stream, prefix, shouldLog)
    {
    }



protected:
    LogStreamBuffer mBuffer;
};


template<class S>
class LogStream : protected LogStreamBase, public std::ostream
{
public:
    //! \brief Creates a LogStreamConsumer which logs messages with level severity.
    //!  Reportable severity determines if the messages are severe enough to be logged.
    LogStream(S reportableSeverity, S severity)
            : LogStreamBase(severityOstream(severity), severityPrefix(severity), severity <= reportableSeverity)
            , std::ostream(&mBuffer) // links the stream buffer with the stream
            , mShouldLog(severity <= reportableSeverity)
            , mSeverity(severity)
    {
    }


    LogStream(LogStream&& other)
            : LogStreamBase(severityOstream(other.mSeverity), severityPrefix(other.mSeverity), other.mShouldLog)
            , std::ostream(&mBuffer) // links the stream buffer with the stream
            , mShouldLog(other.mShouldLog)
            , mSeverity(other.mSeverity)
    {
    }

    void setReportableSeverity(S reportableSeverity)
    {
        mShouldLog = mSeverity <= reportableSeverity;
        mBuffer.setShouldLog(mShouldLog);
    }

    std::string getSystemInfo()
    {
        // prepend timestamp
        std::string time_info;
        std::time_t timestamp = std::time(nullptr);
        tm* tm_local = std::localtime(&timestamp);
        time_info = "["
                    + (tm_local->tm_mon < 10 ? "0" + std::to_string(1 + tm_local->tm_mon):std::to_string(1 + tm_local->tm_mon)) + "/"
                    + (tm_local->tm_mday < 10 ? "0" + std::to_string(tm_local->tm_mday):std::to_string(tm_local->tm_mday)) + "/"
                    + (std::to_string(1900 + tm_local->tm_year)) + "-"
                    + (tm_local->tm_hour < 10 ? "0" + std::to_string(tm_local->tm_hour):std::to_string(tm_local->tm_hour)) + ":"
                    + (tm_local->tm_min < 10 ? "0" + std::to_string(tm_local->tm_min):std::to_string(tm_local->tm_min)) + ":"
                    + (tm_local->tm_sec < 10 ? "0" + std::to_string(tm_local->tm_sec):std::to_string(tm_local->tm_sec))
                    + "] ";
        return time_info;
    }
    static std::string severityPrefix(S severity)
    {
        switch (severity)
        {
            case S::kINTERNAL_ERROR: return "[F] ";
            case S::kERROR: return "[E] ";
            case S::kWARNING: return "[W] ";
            case S::kINFO: return "[I] ";
            case S::kVERBOSE: return "[V] ";
            default: assert(0); return "";
        }
    }
private:
    static std::ostream& severityOstream(S severity)
    {
        return severity >= S::kINFO ? std::cout : std::cerr;
    }

    bool mShouldLog;
    S mSeverity;
};


