#ifndef _log_hpp_
#define _log_hpp_
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <ctime>

namespace sui
{
	//��־����
	class log_basic
	{
	public:
		log_basic() = default;

		virtual void log_output(const std::string& log_data) = 0;
	private:
	};

	//��־���ڱ�׼���������
	class log_bash : public log_basic
	{
	public:
		log_bash() = default;

		void log_output(const std::string& log_data)
		{
			std::unique_lock<std::mutex> lock(_mutex);
			std::cout << log_data << std::endl;
		}

	private:
		std::mutex _mutex;
	};

	const std::string default_path = ".\\log";
	std::string default_file = "log";

	class log_file : public log_basic
	{
	public:
		log_file(const std::string& path = default_path,
			const std::string& file = default_file)
			:_path(path),
			_file(file)
		{
			if (std::filesystem::exists(_path))
			{
				//·������ ֱ�ӷ���
				return;
			}
			else
			{
				//����·��
				try
				{
					std::filesystem::create_directory(_path);
				}
				catch (const std::filesystem::filesystem_error& err)
				{
					std::cout << err.what() << std::endl;
				}
			}
		}

		void log_output(const std::string& log_data)
		{
			std::unique_lock<std::mutex> lock(_mutex);

			std::string _name = _path + (_path.back() == '\\' ? "" : "\\") + _file + ".txt"; //string��֧��ֱ����char��ӣ�ֻ֧����char*���
			//��׷�ӼӲ����ڴ�����ʽ���ļ�
			std::ofstream out(_name, std::ios::out | std::ios::app);
			if (out.is_open())
			{
				out << log_data << std::endl;
				std::cout << "����ɹ�: " << _name << std::endl;
			}
			else
			{
				std::cout << "���ʧ��: " << _name << std::endl;
			}
		}

	private:
		std::mutex _mutex;
		std::string _file;
		std::string _path;
	};

	enum class My_level
	{
		debug,
		info,
		warning,
		error,
		afal
	};

	std::string get_time()
	{
		std::time_t mt = std::time(nullptr);
		std::tm local_time;
		localtime_s(&local_time, &mt);
		char time_str[128];
		snprintf(time_str, sizeof(time_str), "%d/%d/%d - %d : %d : %d", local_time.tm_year + 1900,
			local_time.tm_mon + 1,
			local_time.tm_mday,
			local_time.tm_hour,
			local_time.tm_min,
			local_time.tm_sec);
		return time_str;
	}

	inline std::string Level_to_str(My_level level)
	{
		switch (level)
		{
		case My_level::debug:
			return "debug";

		case My_level::info:
			return "info";

		case My_level::warning:
			return "warning";

		case My_level::error:
			return "error";

		case My_level::afal:
			return "afal";
		default:
			return "unknow";
		}
	}

	//��־������
	class logger
	{
	public:
		logger()
		{
			_ffluent_logg = nullptr;
			Enable_Standard_Logger();
		}

		//�л��ɱ�׼�������ʽ
		void Enable_Standard_Logger()
		{
			if (_ffluent_logg != nullptr)
				delete _ffluent_logg;
			_ffluent_logg = new log_bash;
		}

		void Enable_File_Logger()
		{
			if (_ffluent_logg != nullptr)
				delete _ffluent_logg;
			_ffluent_logg = new log_file;
		}

		//������־��Ϣ����
		class log_message
		{
		public:
			log_message(My_level level, std::string& filename, int line, logger& log)
				:_level(level),
				_filename(filename),
				_line(line),
				_logger(log)
			{
				_curr_time = get_time();
				std::stringstream ss;
				ss << "[" << Level_to_str(_level) << "] "
					<< "[" << _curr_time << "] "
					<< "[" << _filename << "] "
					<< "[" << _line << "] ";
				_output += ss.str();

			}

			template<class T>
			log_message& operator<<(const T& data)
			{
				std::stringstream ss;
				ss << data;
				_output += ss.str();
				return *this;
			}


			~log_message()
			{
				if (!_output.empty())
				{
					_logger._ffluent_logg->log_output(_output);
				}
			}
		private:
			//��ǰ��־ʱ��
			std::string _curr_time;
			//��ǰ�ļ�
			std::string _filename;
			//��ǰ�к�
			int _line;
			//��ǰ��־�ȼ�
			My_level _level;
			//�������Ϣ
			std::string _output;
			logger& _logger;
		};

		log_message operator()(My_level level, std::string filename, int line)
		{
			return log_message(level, filename, line, *this);
		}
	private:
		log_basic* _ffluent_logg;

	};
	inline sui::logger log;

#define LOG(level) sui::log(level, __FILE__, __LINE__)
#define Enable_Bash_Log() sui::log.Enable_Standard_Logger()
#define Enable_File_Log() sui::log.Enable_File_Logger()
}

#endif