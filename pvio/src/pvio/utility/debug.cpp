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
#include <pvio/utility/debug.h>

// Usage: fopen(PVIO_LOG_FILE)
// #define PVIO_LOG_FILE "debug.log", "w"

#if defined(ANDROID) || defined(__ANDROID__)
#define PVIO_LOG_ANDROID
#else
#define PVIO_LOG_CONSOLE
#endif

#ifdef PVIO_LOG_SYSLOG
#include <syslog.h>
#endif
#ifdef PVIO_LOG_ANDROID
#include <andriod/log.h>
#endif
#ifdef PVIO_LOG_WINDOWS
#include <windows.h>
#endif

#define PVIO_LOG_MSGBUF_INIT 512
#define PVIO_LOG_TIMEBUF_MAX 64

namespace pvio {

namespace {

    struct AbstractLogSink {
        virtual ~AbstractLogSink() {
        }
        virtual void log(LogLevel level, const char *message) = 0;
    };

    class LogDispatcher {
        std::vector<std::unique_ptr<AbstractLogSink>> log_sinks;
        LogLevel log_level;
        struct construct_tag {};

        LogDispatcher() = delete;

        template <typename T, typename... Args>
        void add_sink(Args &&... args) {
            log_sinks.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
        }

        void dispatch(LogLevel level, const char *message) {
            if (level < log_level) return;
            for (const auto &sink : log_sinks) {
                sink->log(level, message);
            }
        }

        static LogDispatcher *dispatcher() {
            static std::unique_ptr<LogDispatcher> s_dispatcher = std::make_unique<LogDispatcher>(construct_tag());
            return s_dispatcher.get();
        }

      public:
        LogDispatcher(const construct_tag &);

        static void set_log_level(LogLevel level) {
            dispatcher()->log_level = level;
        }

        static void log_message(LogLevel level, const char *message) {
            dispatcher()->dispatch(level, message);
        }
    };

    static const char *time_string() {
        static char time_string_buf[PVIO_LOG_TIMEBUF_MAX] = {0};
        time_t t = time(nullptr);
        std::strftime(time_string_buf, PVIO_LOG_TIMEBUF_MAX - 1, "%F %T", std::localtime(&t));
        return time_string_buf;
    }

    static const char *level_string(LogLevel level) {
        static const char *level_str_table[] = {
            "debug", "info",
            "notice", "warning",
            "error", "critical",
            "alert", "emergency"};
        return level_str_table[level - PVIO_LOG_DEBUG];
    }

#ifdef PVIO_LOG_CONSOLE
    struct ConsoleLogSink : public AbstractLogSink {
        void log(LogLevel level, const char *message) override {
            fprintf(stderr, "%s - [PVIO][%s] %s\n", time_string(), level_string(level), message);
            fflush(stderr);
        }
    };
#endif

#ifdef PVIO_LOG_FILE
    struct FileLogSink : public AbstractLogSink {
        FILE *logfile;
        FileLogSink() {
            logfile = fopen(PVIO_LOG_FILE);
        }
        ~FileLogSink() {
            if (logfile) {
                fclose(logfile);
            }
        }
        void log(LogLevel level, const char *message) override {
            fprintf(logfile, "%s - [PVIO][%s] %s\n", time_string(), level_string(level), message);
            fflush(logfile);
        }
    };
#endif

#ifdef PVIO_LOG_SYSLOG
    struct SyslogLogSink : public AbstractLogSink {
        SyslogLogSink() {
            setlogmask(LOG_UPTO(LOG_DEBUG));
            openlog(nullptr, LOG_PID, LOG_USER);
        }
        ~SyslogLogSink() {
            closelog();
        }
        static int level_priority(LogLevel level) {
            static int level_priority_map[] = {
                LOG_DEBUG, LOG_INFO,
                LOG_NOTICE, LOG_WARNING,
                LOG_ERR, LOG_CRIT,
                LOG_ALERT, LOG_EMERG};
            return level_priority_map[level - PVIO_LOG_DEBUG];
        }
        void log(LogLevel level, const char *message) override {
            syslog(level_priority(level) | LOG_USER, "%s", message);
        }
    };
#endif

#ifdef PVIO_LOG_ANDROID
    struct AndroidLogSink : public AbstractLogSink {
        static int level_priority(LogLevel level) {
            static int level_priority_map[] = {
                ANDROID_LOG_DEBUG, ANDROID_LOG_INFO,
                ANDROID_LOG_INFO, ANDROID_LOG_WARN,
                ANDROID_LOG_ERROR, ANDROID_LOG_FATAL,
                ANDROID_LOG_FATAL, ANDROID_LOG_FATAL};
            return level_priority_map[level - PVIO_LOG_DEBUG];
        }
        void log(LogLevel level, const char *message) override {
            __android_log_write(level_priority(level), "PVIO", message);
        }
    };
#endif

#if 0
#ifdef PVIO_LOG_WINDOWS
    struct WindowsLogSink : public AbstractLogSink {
        void log(LogLevel level, const char *message) override {
            static char msgbuf[PVIO_LOG_MSGBUF_MAX + 20] = {0};
            snprintf(msgbuf, PVIO_LOG_MSGBUF_MAX + 19, "[PVIO][%s] %s", level_string(level), message);
            OutputDebugStringA(msgbuf);
        }
    };
#endif
#endif

    LogDispatcher::LogDispatcher(const construct_tag &) {
#ifdef PVIO_DEBUG
        log_level = PVIO_LOG_DEBUG;
#else
        log_level = PVIO_LOG_NOTICE;
#endif
#ifdef PVIO_LOG_CONSOLE
        add_sink<ConsoleLogSink>();
#endif
#ifdef PVIO_LOG_FILE
        add_sink<FileLogSink>();
#endif
#ifdef PVIO_LOG_SYSLOG
        add_sink<SyslogLogSink>();
#endif
#ifdef PVIO_LOG_ANDROID
        add_sink<AndroidLogSink>();
#endif
#if 0
#ifdef PVIO_LOG_WINDOWS
        add_sink<WindowsLogSink>();
#endif
#endif
    }

} // namespace

void set_log_level(LogLevel level) {
    LogDispatcher::set_log_level(level);
}

void log_message(LogLevel level, const char *format, ...) {
    static std::vector<char> msgbuf(PVIO_LOG_MSGBUF_INIT);
    va_list vargs1, vargs2;
    va_start(vargs1, format);
    va_copy(vargs2, vargs1);
    int len = vsnprintf(nullptr, 0, format, vargs1);
    va_end(vargs1);
    if (msgbuf.size() < len + 1) {
        msgbuf.resize(len + 1);
    }
    vsnprintf(msgbuf.data(), msgbuf.size(), format, vargs2);
    va_end(vargs2);
    LogDispatcher::log_message(level, msgbuf.data());
}

} // namespace pvio
