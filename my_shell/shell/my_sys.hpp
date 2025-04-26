#pragma once
#include <windows.h>
#include <iostream>
#include <string>
#include <filesystem>
#include <Lmcons.h>
#include <queue>
#include "divide.hpp"
#include <cstring>
#include <list>
#include <unordered_map>
#include <locale>
#include <codecvt>



class my_shell
{
public:
	//初始化函数
	void my_shell_init();
	my_shell();

	//头输出函数
	void my_shell_head();
	
	//用户操作接收函数
	void my_shell_input();

	//操作执行函数
	void user_operator_action(const std::string& arr);

	//进行进程替换函数
	bool process_add(const std::string& temporary_arr, const std::string& parameter_arr);

private:
	std::string my_pwd;
	std::string my_user;
	std::unordered_map<std::wstring, std::wstring> env;
};
