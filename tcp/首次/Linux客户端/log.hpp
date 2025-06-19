#pragma once
#include "mymutex.hpp"
#include <iostream>
#include <string>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <unistd.h>


namespace sui
{
    class LogStrategy
    {
    public:
        ~LogStrategy() = default;

        virtual void SyncLog(const std::string& message) = 0; //chun xv han shu
    private:
    };

    const std::string gsep = "\r\n";
    class ConsoleLogStrategy : public LogStrategy
    {
    public:
        ConsoleLogStrategy()
        {
        }

        void SyncLog(const std::string& message) override
        {
            _mut.lock();
            std::cout << message << gsep;
            _mut.unlock();
        }

        ~ConsoleLogStrategy() = default;
    private:
        my_mutex _mut;
    };

    const std::string defaultpath = "./log";
    const std::string defaultfile = "my.log";
    class FileLogStrategy : public LogStrategy
    {
    public:
        FileLogStrategy(const std::string& path = defaultpath, const std::string& file = defaultfile)
            :_path(path),
            _file(file)
        {
            _mut.lock();
            if (std::filesystem::exists(path))
            {
                _mut.unlock();
                return;
            }
            try
            {
                std::filesystem::create_directories(path);
            }
            catch (const std::filesystem::filesystem_error& err)
            {
                std::cerr << err.what() << std::endl;
            }
            _mut.unlock();
        }

        void SyncLog(const std::string& message) override
        {
            _mut.lock();
            std::string filename = _path + (_path.back() == '/' ? "" : "/") + _file;
            std::ofstream out(filename, std::ios::app);
            if (out.is_open())
            {
                out << message << std::endl;
                out.close();
                _mut.unlock();
            }
        }

        ~FileLogStrategy()
        {
        }

    private:
        my_mutex _mut;
        std::string _path;
        std::string _file;
    };

    //ri zhi deng ji
    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL
    };

    std::string Leve_to_str(LogLevel level)
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
        return "xxx";
    }

    class Logger
    {
    public:
        Logger()
        {
            EnableConsoleLogStrategy();
        }
        void EnableConsoleLogStrategy()
        {
            _ffulsh_strategy = std::make_unique<ConsoleLogStrategy>();
        }
        void EnableFileLogStrategy()
        {
            _ffulsh_strategy = std::make_unique<FileLogStrategy>();
        }

        class LogMessage
        {
        public:
            LogMessage(LogLevel& level, std::string& src_name, int line_number, Logger& logger)
                :_curr_time(GetTimeStamp()),
                _level(level),
                _pid(getpid()),
                _src_name(src_name),
                _line_number(line_number),
                _logger(logger)
            {
                std::stringstream ss;
                ss << "[" << _curr_time << "] "
                    << "[" << Leve_to_str(level) << "] "
                    << "[" << _pid << "] "
                    << "[" << _src_name << "] "
                    << "[" << _line_number << "] "
                    << "-";
                _loginfo = ss.str();
            }

            template <typename T>
            LogMessage& operator<<(const T& info)
            {
                std::stringstream ss;
                ss << info;
                _loginfo += ss.str();
                return *this;
            }

            ~LogMessage()
            {
                if (_logger._ffulsh_strategy)
                {
                    _logger._ffulsh_strategy->SyncLog(_loginfo);
                }
            }

        private:
            std::string _curr_time;
            LogLevel _level;
            pid_t _pid;
            std::string _src_name;
            int _line_number;
            std::string _loginfo;
            Logger& _logger;
        };

        LogMessage operator()(LogLevel level, std::string file_name, int line)
        {
            return LogMessage(level, file_name, line, *this);
        }

        ~Logger()
        {
        }

    private:
        std::unique_ptr<LogStrategy> _ffulsh_strategy;
    };

    Logger logger;

#define LOG(level) sui::logger(level, __FILE__, __LINE__)
#define Enable_Console_Log_Strategy() sui::logger.EnableConsoleLogStrategy()
#define Enable_File_Log_Strategy() sui::logger.EnableFileLogStrategy()
}