#ifndef __LOG_HPP__
#define __LOG_HPP__

#include <iostream>
#include <cstdio>
#include <string>
#include <filesystem> //C++17
#include <sstream>
#include <fstream>
#include <memory>
#include <ctime>
#include <unistd.h>
#include "Mutex.hpp"
// ���ڲ���ģʽ����C++��̬���Ե���־ģ�����
// 2. ˢ�²��� a: ��ʾ����ӡ b:��ָ�����ļ�д��
//  ˢ�²��Ի���

namespace LogModule
{
    using namespace MutexModule;

    const std::string gsep = "\r\n";
   
    class LogStrategy
    {
    public:
        ~LogStrategy() = default;
        virtual void SyncLog(const std::string& message) = 0;
    };

    // ��ʾ����ӡ��־�Ĳ��� : ����
    class ConsoleLogStrategy : public LogStrategy
    {
    public:
        ConsoleLogStrategy()
        {
        }
        void SyncLog(const std::string& message) override
        {
            LockGuard lockguard(_mutex);
            std::cout << message << gsep;
        }
        ~ConsoleLogStrategy()
        {
        }

    private:
        Mutex _mutex;
    };

    // �ļ���ӡ��־�Ĳ��� : ����
    const std::string defaultpath = "./log";
    const std::string defaultfile = "my.log";
    class FileLogStrategy : public LogStrategy
    {
    public:
        FileLogStrategy(const std::string& path = defaultpath, const std::string& file = defaultfile)
            : _path(path),
            _file(file)
        {
            LockGuard lockguard(_mutex);
            if (std::filesystem::exists(_path))
            {
                return;
            }
            try
            {
                std::filesystem::create_directories(_path);
            }
            catch (const std::filesystem::filesystem_error& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        void SyncLog(const std::string& message) override
        {
            LockGuard lockguard(_mutex);

            std::string filename = _path + (_path.back() == '/' ? "" : "/") + _file; // "./log/" + "my.log"
            std::ofstream out(filename, std::ios::app);                              // ׷��д��� ��ʽ��
            if (!out.is_open())
            {
                return;
            }
            out << message << gsep;
            out.close();
        }
        ~FileLogStrategy()
        {
        }

    private:
        std::string _path; // ��־�ļ�����·��
        std::string _file; // ��־�ļ�����
        Mutex _mutex;
    };

    // �γ�һ����������־&&��������Ĳ��ԣ�ѡ��ͬ��ˢ�·�ʽ

    // 1. �γ���־�ȼ�
    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL
    };
    std::string Level2Str(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARNING:
            return "WARNING";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
        }
    }
    std::string GetTimeStamp()
    {
        time_t curr = time(nullptr);
        struct tm curr_tm;
        localtime_r(&curr, &curr_tm);
        char timebuffer[128];
        snprintf(timebuffer, sizeof(timebuffer), "%4d-%02d-%02d %02d:%02d:%02d",
            curr_tm.tm_year + 1900,
            curr_tm.tm_mon + 1,
            curr_tm.tm_mday,
            curr_tm.tm_hour,
            curr_tm.tm_min,
            curr_tm.tm_sec
        );
        return timebuffer;
    }

    // 1. �γ���־ && 2. ���ݲ�ͬ�Ĳ��ԣ����ˢ��
    class Logger
    {
    public:
        Logger()
        {
            EnableConsoleLogStrategy();
        }
        void EnableFileLogStrategy()
        {
            _fflush_strategy = std::make_unique<FileLogStrategy>();
        }
        void EnableConsoleLogStrategy()
        {
            _fflush_strategy = std::make_unique<ConsoleLogStrategy>();
        }

        // ��ʾ����δ����һ����־
        class LogMessage
        {
        public:
            LogMessage(LogLevel& level, std::string& src_name, int line_number, Logger& logger)
                : _curr_time(GetTimeStamp()),
                _level(level),
                _pid(getpid()),
                _src_name(src_name),
                _line_number(line_number),
                _logger(logger)
            {
                // ��־����߲��֣��ϲ�����
                std::stringstream ss;
                ss << "[" << _curr_time << "] "
                    << "[" << Level2Str(_level) << "] "
                    << "[" << _pid << "] "
                    << "[" << _src_name << "] "
                    << "[" << _line_number << "] "
                    << "- ";
                _loginfo = ss.str();
            }
            // LogMessage() << "hell world" << "XXXX" << 3.14 << 1234
            template <typename T>
            LogMessage& operator<<(const T& info)
            {
                // a = b = c =d;
                // ��־���Ұ벿��,�ɱ��
                std::stringstream ss;
                ss << info;
                _loginfo += ss.str();
                return *this;
            }

            ~LogMessage()
            {
                if (_logger._fflush_strategy)
                {
                    _logger._fflush_strategy->SyncLog(_loginfo);
                }
            }

        private:
            std::string _curr_time;
            LogLevel _level;
            pid_t _pid;
            std::string _src_name;
            int _line_number;
            std::string _loginfo; // �ϲ�֮��һ����������Ϣ
            Logger& _logger;
        };

        // �������д�ɷ�����ʱ����
        LogMessage operator()(LogLevel level, std::string name, int line)
        {
            return LogMessage(level, name, line, *this);
        }
        ~Logger()
        {
        }

    private:
        std::unique_ptr<LogStrategy> _fflush_strategy;
    };

    // ȫ����־����
    Logger logger;

    // ʹ�ú꣬���û���������ȡ�ļ������к�
#define LOG(level) logger(level, __FILE__, __LINE__)
#define Enable_Console_Log_Strategy() logger.EnableConsoleLogStrategy()
#define Enable_File_Log_Strategy() logger.EnableFileLogStrategy()
}

#endif