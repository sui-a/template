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
	//��ʼ������
	void my_shell_init();
	my_shell();

	//ͷ�������
	void my_shell_head();
	
	//�û��������պ���
	void my_shell_input();

	//����ִ�к���
	void user_operator_action(const std::string& arr);

	//���н����滻����
	bool process_add(const std::string& temporary_arr, const std::string& parameter_arr);

private:
	std::string my_pwd;
	std::string my_user;
	std::unordered_map<std::wstring, std::wstring> env;
};
