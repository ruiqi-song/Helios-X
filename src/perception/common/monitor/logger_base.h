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

class LoggerBase{
public:
    //!
    //! The severity corresponding to a log message.
    //!
    enum class Severity : int{
        kINTERNAL_ERROR = 0, //!< An internal error has occurred. Execution is unrecoverable.
        kERROR = 1,          //!< An application error has occurred.
        kWARNING = 2,        //!< An application error has been discovered, but TensorRT has recovered or fallen back to a default.
        kINFO = 3,           //!< Informational messages with instructional information.
        kVERBOSE = 4,        //!< Verbose messages with debugging information.
    };

    //!
    //! A callback implemented by the application to handle logging messages;
    //!
    //! \param severity The severity of the message.
    //! \param msg The log message, null terminated.
    //!
    virtual void log(Severity severity, const char* msg)  = 0;

    //!
    //! A callback implemented by the application to record logging messages;
    //!
    //! \param severity The severity of the message.
    //! \param msg The log message, null terminated.
    //!
    virtual void record(Severity severity, const char* msg)  = 0;


    virtual ~LoggerBase() {}
};
