#include "divide.hpp"

std::queue<std::string> string_space_divide(const std::string& arr)
{
    std::queue<std::string> my_que;
    std::string temporary_arr;
    for (auto it = arr.begin(); it != arr.end(); ++it)
    {
        if (*it == ' ' && !temporary_arr.empty())
        {
            my_que.push(temporary_arr); //推送到队列
            temporary_arr.clear(); // 清空字符串
        }
        else if (*it != ' ')
        {
            temporary_arr += *it;
        }
    }

    //如果最后一个变量不是空格，则
    if (!temporary_arr.empty()) {
        my_que.push(temporary_arr);
    }

    return my_que;
}