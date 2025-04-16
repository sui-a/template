#include<iostream>
#include<cstring>
#include<cstdio>
#include<unistd.h>

template <typename T>
void fun(T x, T y)
{
    int z = (int)((double)(x / y)) * 100;
    char w[100];
    char t[5] = "|/-\\";
    static int p = 0;
    memset(w, 0, sizeof(w));

    for (int i = 0; i < z && i < sizeof(w); i++)
    {
        w[i] = '#';
    }
    printf("[%-100s][%-3d%%][%c]", w, z, t[p % 4]);
    if (z >= 100)
        printf("/n");
    else
        printf("/r");
    std::cout.flush();
    p++;
}

int main()
{
    for (int i = 0; i <= 100; i++)
    {
        fun(i, 100);
        sleep(3);
    }
    return 0;
}