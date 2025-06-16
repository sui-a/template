#pragma once
#include "log.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <map>


namespace sui
{
	//存储字典默认路径和文件名
	std::string default_dictionary_path = ".\\";
	std::string default_dictionary_file = "dictionary.txt";


	//创建类
	class my_direction
	{
	public:
		my_direction(std::string& path = default_dictionary_path, std::string& file = default_dictionary_file)
			:_path(path),
			_file(file)
		{
			std::string name = default_dictionary_path + (_path.back() == '\\' ? "" : "\\") + _file;
			std::ifstream dir_in(name);
			std::string data;
			char delimiter = ':';
			LOG(My_level::info) << "开始读取:" << name;
			while (std::getline(dir_in, data))
			{
				std::vector<std::string> deli_data;
				std::string deli_data_son;
				int is_to = 0;
				for(int i = 0; i <= data.size(); i++)
				{
					if (i == data.size() || data[i] == delimiter)
					{
						deli_data.push_back(deli_data_son);
						deli_data_son.clear();
					}
					else
					{
						deli_data_son += data[i];
					}
				}
				change[deli_data[0]] = deli_data[1];
				change[deli_data[1]] = deli_data[0];
			}
		}

		bool get_change(const std::string& key, std::string& results)
		{
			
			if (change.find(key) != change.end()) 
			{
				results = change[key];
				return true;
			}
			return false;
		}

		~my_direction()
		{ }

	private:
		std::string _path;
		std::string _file;
		//中文转英文
		std::map<std::string, std::string> change;
	};

}