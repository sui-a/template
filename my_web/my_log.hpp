#pragma once 
#include <iostream>
#include <unistd.h>
#include <string>
#include <filesystem>
#include <fstream>
#include "Mutex.hpp"


namespace sui
{
    //策略结构日志输出
    //基类
    class log_base
    {
    public:
        //初始化
        log_base() = default;

        //日志输出
        virtual void log_output(std::string& log_data)
        {
            //默认不输出
        }
        ~log_base() = default;
    };


    //规定结束符
    std::string log_end = "\r\n";

    //终端输出
    class log_terminal : public  log_base
    {
    public:
        log_terminal() = default;
        ~log_terminal() = default;

        void log_output(std::string& log_data) override
        {
            LockGuard lockguard(_mut);
            std::cout << log_data << log_end;
        }
    private:
        Mutex _mut;
    };


    //设置默认文件输出地址
    std::string filepath = "./log";
    std::string filename = "log.txt";

    //文件输出
    class log_file : public  log_base
    {
    public:
        log_file(std::string path = filepath, std::string name = filename)
        :_path(path),
        _name(name)
        {
            //上所防止多次创建
            LockGuard lockguard(_mut);
            //进行路径判断是否存在
            if(std::filesystem::exists(_path))
            {
                //路径存在，不进行操作
            }
            else
            {
                //进行路径创建
                try
                {
                    std::filesystem::create_directories(path);
                }
                catch(...)
                {
                    std::cerr << "日志路径创建异常" << '\n';
                }
            }
        }
        ~log_file() = default;

        void log_output(std::string& log_data) override
        {
            LockGuard lockguard(_mut);
            //进行日志输出
            //追加写入方式打开文件
            std::string filename = _path + (_path.back() == '/' ? "" : "/") + _name;
            std::ofstream file(filename, std::ios::app);
            //判断文件是否打开
            if(file.is_open())
            {
                file << log_data << log_end;  // 写入数据并换行
                file.close();
                return;
            }
            //没有打开文件
            std::cout << "文件打开失败" << std::endl;
            return;
        }
    private:
        Mutex _mut;
        std::string _name;
        std::string _path;
    };

    //设置日志等级
    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL
    };
    //翻译
    std::string Level2Str(LogLevel& level)
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
    //获取时间
    std::string get_current_time() 
    {
        // 获取当前系统时间
        auto now = std::chrono::system_clock::now();
        std::time_t time = std::chrono::system_clock::to_time_t(now);
        
        // 转换为本地时间并格式化
        std::tm tm = *std::localtime(&time);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");  // 格式示例：2025-06-29 14:30:45
        return ss.str();
    }


    //日志控制本体
    class Logger
    {
    public:
        Logger()
        {
            //切换成默认的输出形式
            Log_to_terminal();
        }

        //切换成默认终端输出模式
        void Log_to_terminal()
        {
            _fflush_stratgy = std::make_unique<log_terminal>();
        }

        //切换成文件输出模式
        void Log_to_file()
        {
            _fflush_stratgy = std::make_unique<log_file>();
        }

        //创建类，以支持流操作
        class Log_message
        {
        public:
            Log_message(Logger* this_logger, LogLevel& level, std::string& file_name, int& file_line)
            :_logger(this_logger)
            {
                _total_data += "[" + Level2Str(level) + "]"
                            + "[" + get_current_time() + "]"
                            + "[" + std::to_string(getpid()) + "]"
                            + "[" + file_name + "]"
                            + "[" + std::to_string(file_line) + "] ";
            }
            ~Log_message()
            {
                _logger->_fflush_stratgy->log_output(_total_data);
            }

            Log_message& operator<<(std::string dat)
            {
                _total_data += dat;
                return *this;
            }

        private:
            //输出消息
            std::string _total_data;
            Logger* _logger;
        };

        Log_message operator()(LogLevel level, std::string file_name, int file_line)
        {
            return Log_message(this, level, file_name, file_line);
        }


    private:
        std::unique_ptr<log_base> _fflush_stratgy;
    };


    Logger logger;

#define LOG(level) sui::logger(level, __FILE__, __LINE__)
#define LOG_TO_TERMINAL() sui::logger.Log_to_terminal()
#define LOG_TO_FILE() sui::logger.Log_to_file()
}