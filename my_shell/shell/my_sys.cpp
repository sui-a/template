#include "my_sys.hpp"

void my_shell::my_shell_init()
{
    //初始化当前路径变量
    std::filesystem::path pwd = std::filesystem::current_path();
    //判断错误
    if (pwd.empty())
    {
        std::cout << "获取环境错误" << std::endl;
        exit(1);
    }
    //切换存储风格
    my_pwd = pwd.string();
    //进行引号去除
    if (my_pwd.front() == '"') {
        my_pwd.erase(0, 1);
    }
    if (my_pwd.back() == '"') {
        my_pwd.pop_back();
    }


    // 初始化当前用户环境变量
    char username[UNLEN + 1];
    DWORD size = UNLEN + 1;
    // 调用 GetUserNameA 获取当前用户名
    if (GetUserNameA(username, &size)) {
        // 将 C 风格字符串转换为 std::string
        my_user = username;
    }
    else {
        std::cerr << "获取用户名失败，错误代码: " << GetLastError() << std::endl;
        my_user = "";
    }

   //获取全部的系统变量
    LPWCH env_strings = GetEnvironmentStringsW();
    if (env_strings == nullptr)
    {
        std::wcerr << L"Failed to retrieve environment strings." << std::endl;
    }

    LPWCH current = env_strings;
    while (*current)
    {
        std::wstring entry(current);
        size_t pos = entry.find(L'=');
        if (pos != std::wstring::npos)
        {
            std::wstring key = entry.substr(0, pos);
            std::wstring value = entry.substr(pos + 1);
            env[key] = value;
        }
        current += entry.size() + 1;
    }

    FreeEnvironmentStringsW(env_strings);

   
}

my_shell::my_shell()
{
    my_shell_init();
}

void my_shell::my_shell_head()
{
	std::cout << "[" << my_pwd << " " << my_user << " ]@: ";
}

void my_shell::my_shell_input()
{
	std::string my_input;
	std::getline(std::cin, my_input);  // 读取整行输入，包括空格
    user_operator_action(my_input);
}

bool my_shell::process_add(const std::string& temporary_arr, const std::string& parameter_arr)
{
    // 构建命令行字符串，包含可执行文件路径和参数
    std::string commandLine;
    if(temporary_arr == "cls" ||
        temporary_arr == "cd" ||
        temporary_arr == "copy" ||
        temporary_arr == "del" || 
        temporary_arr == "dir" ||
        temporary_arr == "exit" ||
        temporary_arr == "md" ||
        temporary_arr == "rd")
    {
        commandLine =   "cmd.exe /C \"" + temporary_arr + "\" " + parameter_arr;
    }
    else
    {
        commandLine = "\"" + temporary_arr + "\" " + parameter_arr;
    }

    // 将 std::string 转换为 char*，CreateProcessA 需要可修改的字符串
    char* cmdLine = new char[commandLine.size() + 1];
    strcpy_s(cmdLine, commandLine.size() + 1, commandLine.c_str());

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    // 创建子进程
    BOOL success = CreateProcessA(
        NULL,           // 应用程序名称
        cmdLine,        // 命令行
        NULL,           // 进程安全属性
        NULL,           // 线程安全属性
        FALSE,          // 是否继承句柄
        0,              // 创建标志
        NULL,           // 环境变量
        NULL,           // 当前目录
        &si,            // 启动信息
        &pi             // 进程信息
    );

    delete[] cmdLine; // 释放分配的内存

    if (!success) {
        std::cerr << "CreateProcess failed (" << GetLastError() << ")." << std::endl;
        return false;
    }

    // 等待子进程结束
    WaitForSingleObject(pi.hProcess, INFINITE);

    // 获取子进程的退出代码
    DWORD exitCode;
    if (!GetExitCodeProcess(pi.hProcess, &exitCode)) {
        std::cerr << "Failed to get exit code." << std::endl;
    }

    // 关闭进程和线程的句柄
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    //重新初始化
    my_shell_init();

    return true;
}

void my_shell::user_operator_action(const std::string& arr)
{
    if (!arr.empty())
    {
        std::string temporary_arr;
        std::string parameter_arr;
        std::string test;
        bool operator_judge = false;
        //判断是否需要宏替换
        bool replace_judge = false;
        //进行输入遍历
        for (auto it = arr.begin(); it != arr.end(); ++it)
        {
            if (!operator_judge)
            {
                if (*it == ' ' || it + 1 == arr.end())
                {
                    operator_judge = true;
                    if (replace_judge)
                    {
                        
                        int size_needed = MultiByteToWideChar(CP_UTF8, 0, temporary_arr.c_str(),
                            (int)temporary_arr.size(), nullptr, 0);
                        std::wstring wstr(size_needed, 0);
                        MultiByteToWideChar(CP_UTF8, 0, temporary_arr.c_str(), 
                            (int)temporary_arr.size(), &wstr[0], size_needed);

                        auto is = env.find(wstr);
                        if (is != env.end()) {
                            // 获取环境变量值的宽字符长度
                            int size_needed = WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, nullptr, 0, nullptr, nullptr);
                            if (size_needed > 0) {
                                // 分配足够的空间存储转换后的字符串
                                temporary_arr.resize(size_needed - 1, 0); // 不包括终止符
                                // 将宽字符转换为多字节字符串
                                WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, &temporary_arr[0], size_needed, nullptr, nullptr);
                            }
                        }
                        else {
                            temporary_arr = '$' + temporary_arr;
                        }
                        replace_judge = false;
                    }
                    if (*it != ' ' && it + 1 == arr.end())
                    {
                        temporary_arr += *it;
                    }
                }
                else if (*it != ' ')
                {
                    if (temporary_arr.empty() && *it == '$')
                    {
                        replace_judge = true;
                    }
                    else
                    {
                        temporary_arr += *it;
                    }
                }
                continue;
            }

            if (operator_judge)
            {
                if (replace_judge)
                {
                    if (*it == ' ' || it + 1 == arr.end())
                    {
                        if (*it != ' ' && it + 1 == arr.end())
                        {
                            test.push_back(*it);
                        }
                        int size_needed = MultiByteToWideChar(CP_UTF8, 0, test.c_str(),
                            (int)test.size(), nullptr, 0);
                        std::wstring wstr(size_needed, 0);
                        MultiByteToWideChar(CP_UTF8, 0, test.c_str(),
                            (int)test.size(), &wstr[0], size_needed);

                        auto is = env.find(wstr);
                        if (is != env.end()) 
                        {
                            // 获取环境变量值的宽字符长度
                            int size_needed = WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, nullptr, 0, nullptr, nullptr);
                            if (size_needed > 0) {
                                // 分配足够的空间存储转换后的字符串
                                std::string ass(size_needed - 1, 0); // 不包括终止符
                                // 将宽字符转换为多字节字符串
                                WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, &ass[0], size_needed, nullptr, nullptr);
                                parameter_arr += ass;
                                
                            }
                        }
                        else 
                        {
                            parameter_arr = parameter_arr + '$' + test;
                            test.clear();
                        }
                        replace_judge = false;
                    }
                    else
                    {
                        test.push_back(*it);
                    }
                }
                else
                {
                    if(*it != '$')
                    {
                        parameter_arr.push_back(*it);
                    }
                    else
                    {
                        replace_judge = true;
                    }

                }
            }
        }
        std::cout <<  "temporary    " << temporary_arr << std::endl;
        std::cout << "parameter     " << parameter_arr << std::endl;


        if (temporary_arr == "exit")
        {
            exit(0);
        }
        if (temporary_arr == "env")
        {
            for (const auto& pair : env)
            {
                std::wcout << pair.first << L"=" << pair.second << std::endl;
            }
        }

        bool my_process_action = process_add(temporary_arr, parameter_arr);
    }
}