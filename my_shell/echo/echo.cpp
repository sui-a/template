#include <iostream>

int main(int argv, char** argc)
{
    for (int i = 1; i < argv; ++i)
    {
        std::cout << argc[i];
        if (i < argv - 1)
            std::cout << " ";
    }
    std::cout << std::endl;
    return 0;
}