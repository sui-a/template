#pragma once 
//设置无法复制继承类

namespace sui
{
    class No_copy
    {
    public:
        const No_copy& operator=(No_copy cop) = delete;
        No_copy(No_copy& cop) = delete;
    };
}