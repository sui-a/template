#include "read.hpp"

sui::my_read my_cl_read(std::string("fifo"));

void signal_handler(int signum)
{
    std::cout << "shut fifo" << std::endl;
    my_cl_read.stop();
    exit(signum);
}

int main()
{

    //callback function after registering the reveive signal
    signal(SIGINT, signal_handler);

    while (true)
    {
        const std::string dat = my_cl_read.work();
        if (dat.empty())
        {
            continue;
        }
        std::cout << "client say:   " << dat << std::endl;
    }
    return 0;
}