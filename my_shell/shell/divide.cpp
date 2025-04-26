#include "divide.hpp"

std::queue<std::string> string_space_divide(const std::string& arr)
{
    std::queue<std::string> my_que;
    std::string temporary_arr;
    for (auto it = arr.begin(); it != arr.end(); ++it)
    {
        if (*it == ' ' && !temporary_arr.empty())
        {
            my_que.push(temporary_arr); //���͵�����
            temporary_arr.clear(); // ����ַ���
        }
        else if (*it != ' ')
        {
            temporary_arr += *it;
        }
    }

    //������һ���������ǿո���
    if (!temporary_arr.empty()) {
        my_que.push(temporary_arr);
    }

    return my_que;
}