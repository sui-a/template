#include "my_sys.hpp"

void my_shell::my_shell_init()
{
    //��ʼ����ǰ·������
    std::filesystem::path pwd = std::filesystem::current_path();
    //�жϴ���
    if (pwd.empty())
    {
        std::cout << "��ȡ��������" << std::endl;
        exit(1);
    }
    //�л��洢���
    my_pwd = pwd.string();
    //��������ȥ��
    if (my_pwd.front() == '"') {
        my_pwd.erase(0, 1);
    }
    if (my_pwd.back() == '"') {
        my_pwd.pop_back();
    }


    // ��ʼ����ǰ�û���������
    char username[UNLEN + 1];
    DWORD size = UNLEN + 1;
    // ���� GetUserNameA ��ȡ��ǰ�û���
    if (GetUserNameA(username, &size)) {
        // �� C ����ַ���ת��Ϊ std::string
        my_user = username;
    }
    else {
        std::cerr << "��ȡ�û���ʧ�ܣ��������: " << GetLastError() << std::endl;
        my_user = "";
    }

   //��ȡȫ����ϵͳ����
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
	std::getline(std::cin, my_input);  // ��ȡ�������룬�����ո�
    user_operator_action(my_input);
}

bool my_shell::process_add(const std::string& temporary_arr, const std::string& parameter_arr)
{
    // �����������ַ�����������ִ���ļ�·���Ͳ���
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

    // �� std::string ת��Ϊ char*��CreateProcessA ��Ҫ���޸ĵ��ַ���
    char* cmdLine = new char[commandLine.size() + 1];
    strcpy_s(cmdLine, commandLine.size() + 1, commandLine.c_str());

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    // �����ӽ���
    BOOL success = CreateProcessA(
        NULL,           // Ӧ�ó�������
        cmdLine,        // ������
        NULL,           // ���̰�ȫ����
        NULL,           // �̰߳�ȫ����
        FALSE,          // �Ƿ�̳о��
        0,              // ������־
        NULL,           // ��������
        NULL,           // ��ǰĿ¼
        &si,            // ������Ϣ
        &pi             // ������Ϣ
    );

    delete[] cmdLine; // �ͷŷ�����ڴ�

    if (!success) {
        std::cerr << "CreateProcess failed (" << GetLastError() << ")." << std::endl;
        return false;
    }

    // �ȴ��ӽ��̽���
    WaitForSingleObject(pi.hProcess, INFINITE);

    // ��ȡ�ӽ��̵��˳�����
    DWORD exitCode;
    if (!GetExitCodeProcess(pi.hProcess, &exitCode)) {
        std::cerr << "Failed to get exit code." << std::endl;
    }

    // �رս��̺��̵߳ľ��
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    //���³�ʼ��
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
        //�ж��Ƿ���Ҫ���滻
        bool replace_judge = false;
        //�����������
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
                            // ��ȡ��������ֵ�Ŀ��ַ�����
                            int size_needed = WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, nullptr, 0, nullptr, nullptr);
                            if (size_needed > 0) {
                                // �����㹻�Ŀռ�洢ת������ַ���
                                temporary_arr.resize(size_needed - 1, 0); // ��������ֹ��
                                // �����ַ�ת��Ϊ���ֽ��ַ���
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
                            // ��ȡ��������ֵ�Ŀ��ַ�����
                            int size_needed = WideCharToMultiByte(CP_UTF8, 0, is->second.c_str(), -1, nullptr, 0, nullptr, nullptr);
                            if (size_needed > 0) {
                                // �����㹻�Ŀռ�洢ת������ַ���
                                std::string ass(size_needed - 1, 0); // ��������ֹ��
                                // �����ַ�ת��Ϊ���ֽ��ַ���
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