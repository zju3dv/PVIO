/**************************************************************************
* This file is part of PVIO
*
* Copyright (c) ZJU-SenseTime Joint Lab of 3D Vision. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#ifndef PVIO_DEBUG_H
#define PVIO_DEBUG_H

#include <pvio/common.h>

namespace pvio {

enum LogLevel {
    PVIO_LOG_DEBUG = -1,  /**< debug message                      **/
    PVIO_LOG_INFO = 0,    /**< informational message              **/
    PVIO_LOG_NOTICE = 1,  /**< normal, but significant, condition **/
    PVIO_LOG_WARNING = 2, /**< warning conditions                 **/
    PVIO_LOG_ERR = 3,     /**< error conditions                   **/
    PVIO_LOG_CRIT = 4,    /**< critical conditions                **/
    PVIO_LOG_ALERT = 5,   /**< action must be taken immediately   **/
    PVIO_LOG_EMERG = 6    /**< system is unusable                 **/
};

void set_log_level(LogLevel level);
void log_message(LogLevel level, const char *format, ...);

inline void runtime_assert(bool condition, const std::string &message, bool shutdown = true) {
    if (!condition) {
        log_message(PVIO_LOG_ERR, "%s", message.c_str());
        if (shutdown) {
            abort();
        }
    }
}

} // namespace pvio

#endif // PVIO_DEBUG_H
