/******************************************************************************
 * Copyright 2022 The Helios-X Authors. All Rights Reserved.
 * Author: Ricky Song
 * Time: 2022-2-10
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

#include<fstream>
#include<iostream>

#include "NvInferRuntime.h"
#include "log_stream.h"

class GLogger : public LoggerBase
{
public:

    GLogger(GSeverity severity = GSeverity::kWARNING)
            : mReportableSeverity(severity)
    {
    }

    //! \brief Implementation of the LoggerBase::log() virtual method
    void log(GSeverity severity, const char* msg) override
    {
        LogStream<GSeverity>(mReportableSeverity, severity) << "[GENERAL] " << std::string(msg) << std::endl;
    }

    //! \brief Implementation of the LoggerBase::record() virtual method
    void record(GSeverity severity, const char* msg) override
    {
        //LogStream<GSeverity>(mReportableSeverity, severity) << "[GENERAL] " << std::string(msg) << std::endl;
        auto log = LogStream<GSeverity>(mReportableSeverity, severity);
        std::string time = log.getSystemInfo();
        std::string sev = log.severityPrefix(severity);
        std::ofstream writer;
        writer.open("/home/ricky/waytous_server/src/perception/io/log/2022-02-14-log-record.txt", std::ios::app);
        writer << time << sev << std::string(msg) << std::endl;
        writer.close(); //关闭Test.txt文件
    }

    //! \brief Method for controlling the verbosity of logging output
    void setReportableSeverity(GSeverity severity)
    {
        mReportableSeverity = severity;
    }

    GSeverity getReportableSeverity() const
    {
        return mReportableSeverity;
    }

private:
    //! \brief returns an appropriate string for prefixing a log message with the given severity
    static const char* severityPrefix(GSeverity severity)
    {
        switch (severity)
        {
            case GSeverity::kINTERNAL_ERROR: return "[F] ";
            case GSeverity::kERROR: return "[E] ";
            case GSeverity::kWARNING: return "[W] ";
            case GSeverity::kINFO: return "[I] ";
            case GSeverity::kVERBOSE: return "[V] ";
            default: assert(0); return "";
        }
    }

    //! \brief returns an appropriate output stream (cout or cerr) to use with the given severity
    static std::ostream& severityOstream(GSeverity severity)
    {
        return severity >= GSeverity::kINFO ? std::cout : std::cerr;
    }

    GSeverity mReportableSeverity;
};

class TRTLogger : public nvinfer1::ILogger
{
public:
    TRTLogger(TRTSeverity severity = TRTSeverity::kWARNING)
            : mReportableSeverity(severity)
    {
    }

    //! \brief Represents the state of a given test
    enum class TestResult
    {
        kRUNNING, //!< The test is running
        kPASSED,  //!< The test passed
        kFAILED,  //!< The test failed
        kWAIVED   //!< The test was waived
    };

    //! \brief Forward-compatible method for retrieving the nvinfer::ILogger associated with this Logger
    //! \return The nvinfer1::ILogger associated with this Logger
    nvinfer1::ILogger& getTRTLogger()
    {
        return *this;
    }

    //! \brief Implementation of the nvinfer1::ILogger::log() virtual method

    void log(nvinfer1::ILogger::Severity severity, nvinfer1::AsciiChar const * msg) noexcept override
    {
        LogStream<TRTSeverity>(mReportableSeverity, severity) << "[TRT] " << std::string(msg) << std::endl;
    }






    //! \brief Method for controlling the verbosity of logging output
    void setReportableSeverity(TRTSeverity severity)
    {
        mReportableSeverity = severity;
    }

    //! \brief Opaque handle that holds logging information for a particular test
    class TestAtom
    {
    public:
        TestAtom(TestAtom&&) = default;

    private:
        friend class TRTLogger;

        TestAtom(bool started, const std::string& name, const std::string& cmdline)
                : mStarted(started)
                , mName(name)
                , mCmdline(cmdline)
        {
        }

        bool mStarted;
        std::string mName;
        std::string mCmdline;
    };

    //! \brief Define a test for logging
    //! \return a TestAtom that can be used in Logger::reportTest{Start,End}().
    static TestAtom defineTest(const std::string& name, const std::string& cmdline)
    {
        return TestAtom(false, name, cmdline);
    }

    //! \brief A convenience overloaded version of defineTest() that accepts an array of command-line arguments as input
    //! \return a TestAtom that can be used in Logger::reportTest{Start,End}().
    static TestAtom defineTest(const std::string& name, int argc, char const* const* argv)
    {
        auto cmdline = genCmdlineString(argc, argv);
        return defineTest(name, cmdline);
    }

    //! \brief Report that a test has started.
    static void reportTestStart(TestAtom& testAtom)
    {
        reportTestResult(testAtom, TestResult::kRUNNING);
        assert(!testAtom.mStarted);
        testAtom.mStarted = true;
    }

    //! \brief Report that a test has ended.
    static void reportTestEnd(const TestAtom& testAtom, TestResult result)
    {
        assert(result != TestResult::kRUNNING);
        assert(testAtom.mStarted);
        reportTestResult(testAtom, result);
    }

    static int reportPass(const TestAtom& testAtom)
    {
        reportTestEnd(testAtom, TestResult::kPASSED);
        return EXIT_SUCCESS;
    }

    static int reportFail(const TestAtom& testAtom)
    {
        reportTestEnd(testAtom, TestResult::kFAILED);
        return EXIT_FAILURE;
    }

    static int reportWaive(const TestAtom& testAtom)
    {
        reportTestEnd(testAtom, TestResult::kWAIVED);
        return EXIT_SUCCESS;
    }

    static int reportTest(const TestAtom& testAtom, bool pass)
    {
        return pass ? reportPass(testAtom) : reportFail(testAtom);
    }

    Severity getReportableSeverity() const
    {
        return mReportableSeverity;
    }

private:
    //! \brief returns an appropriate string for prefixing a log message with the given severity
    static const char* severityPrefix(TRTSeverity severity)
    {
        switch (severity)
        {
            case TRTSeverity::kINTERNAL_ERROR: return "[F] ";
            case TRTSeverity::kERROR: return "[E] ";
            case TRTSeverity::kWARNING: return "[W] ";
            case TRTSeverity::kINFO: return "[I] ";
            case TRTSeverity::kVERBOSE: return "[V] ";
            default: assert(0); return "";
        }
    }

    //! \brief returns an appropriate string for prefixing a test result message with the given result
    static const char* testResultString(TestResult result)
    {
        switch (result)
        {
            case TestResult::kRUNNING: return "RUNNING";
            case TestResult::kPASSED: return "PASSED";
            case TestResult::kFAILED: return "FAILED";
            case TestResult::kWAIVED: return "WAIVED";
            default: assert(0); return "";
        }
    }

    //! \brief returns an appropriate output stream (cout or cerr) to use with the given severity
    static std::ostream& severityOstream(TRTSeverity severity)
    {
        return severity >= TRTSeverity::kINFO ? std::cout : std::cerr;
    }

    //! \brief method that implements logging test results
    static void reportTestResult(const TestAtom& testAtom, TestResult result)
    {
        severityOstream(TRTSeverity::kINFO) << "&&&& " << testResultString(result) << " " << testAtom.mName << " # "
                                         << testAtom.mCmdline << std::endl;
    }

    //! \brief generate a command line string from the given (argc, argv) values
    static std::string genCmdlineString(int argc, char const* const* argv)
    {
        std::stringstream ss;
        for (int i = 0; i < argc; i++)
        {
            if (i > 0)
                ss << " ";
            ss << argv[i];
        }
        return ss.str();
    }

    Severity mReportableSeverity;
};


//// Logger for TensorRT info/warning/errors
//class LoggerWarning : public nvinfer1::ILogger
//{
//public:
//    LoggerWarning(Severity severity = Severity::kWARNING) : reportableSeverity(severity)
//    {
//    }
//
//    void log(Severity severity, const char* msg) override
//    {
//        // suppress messages with severity enum value greater than the reportable
//        if (severity > reportableSeverity)
//            return;
//
//        switch (severity)
//        {
//            case Severity::kINTERNAL_ERROR:
//                std::cerr << "INTERNAL_ERROR: ";
//                break;
//            case Severity::kERROR:
//                std::cerr << "ERROR: ";
//                break;
//            case Severity::kWARNING:
//                std::cerr << "WARNING: ";
//                break;
//            case Severity::kINFO:
//                std::cerr << "INFO: ";
//                break;
//            default:
//                std::cerr << "UNKNOWN: ";
//                break;
//        }
//        std::cerr << msg << std::endl;
//    }
//
//    Severity reportableSeverity;
//};

namespace
{

//!
//! \brief produces a LogStreamConsumer object that can be used to log messages of severity kVERBOSE
//!
//! Example usage:
//!
//!     LOG_VERBOSE(logger) << "hello world" << std::endl;
//!

    inline LogStream<GSeverity>LOG_VERBOSE(const GLogger & logger)
    {
        return LogStream<GSeverity>(logger.getReportableSeverity(), GSeverity ::kVERBOSE);
    }

//!
//! \brief produces a LogStreamConsumer object that can be used to log messages of severity kINFO
//!
//! Example usage:
//!
//!     LOG_INFO(logger) << "hello world" << std::endl;
//!
    inline LogStream<GSeverity> LOG_INFO(const GLogger& logger)
    {
        return LogStream<GSeverity>(logger.getReportableSeverity(), GSeverity::kINFO);
    }

//!
//! \brief produces a LogStreamConsumer object that can be used to log messages of severity kWARNING
//!
//! Example usage:
//!
//!     LOG_WARN(logger) << "hello world" << std::endl;
//!
    inline LogStream<GSeverity> LOG_WARN(const GLogger& logger)
    {
        return LogStream<GSeverity>(logger.getReportableSeverity(), GSeverity::kWARNING);
    }

//!
//! \brief produces a LogStreamConsumer object that can be used to log messages of severity kERROR
//!
//! Example usage:
//!
//!     LOG_ERROR(logger) << "hello world" << std::endl;
//!
    inline LogStream<GSeverity> LOG_ERROR(const GLogger& logger)
    {
        return LogStream<GSeverity>(logger.getReportableSeverity(), GSeverity::kERROR);
    }

//!
//! \brief produces a LogStreamConsumer object that can be used to log messages of severity kINTERNAL_ERROR
//         ("fatal" severity)
//!
//! Example usage:
//!
//!     LOG_FATAL(logger) << "hello world" << std::endl;
//!
    inline LogStream<GSeverity> LOG_FATAL(const GLogger& logger)
    {
        return LogStream<GSeverity>(logger.getReportableSeverity(), GSeverity::kINTERNAL_ERROR);
    }

} // anonymous namespace


