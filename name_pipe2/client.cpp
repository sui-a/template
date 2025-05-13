#include "write.hpp"

int main()
{
    sui::my_write my_cl_write(std::string("fifo"));

    std::string dat;
    while (true)
    {
        std::getline(std::cin, dat);
        bool a = my_cl_write.work(dat);
        if (!a)
        {
            std::cerr << "work error" << std::endl;
        }
    }
    return 0;
}