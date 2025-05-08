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

#include "log.h"


GLogger hLogger{GLogger::Severity::kINFO};
TRTLogger trtLogger{TRTLogger::Severity::kINFO};
LogStream<GSeverity> gLogVerbose{LOG_VERBOSE(hLogger)};
LogStream<GSeverity> gLogInfo{LOG_INFO(hLogger)};
LogStream<GSeverity> gLogWarning{LOG_WARN(hLogger)};
LogStream<GSeverity> gLogError{LOG_ERROR(hLogger)};
LogStream<GSeverity> gLogFatal{LOG_FATAL(hLogger)};

void setReportableSeverity(GLogger::Severity severity)
{
    hLogger.setReportableSeverity(severity);
    gLogVerbose.setReportableSeverity(severity);
    gLogInfo.setReportableSeverity(severity);
    gLogWarning.setReportableSeverity(severity);
    gLogError.setReportableSeverity(severity);
    gLogFatal.setReportableSeverity(severity);
}

