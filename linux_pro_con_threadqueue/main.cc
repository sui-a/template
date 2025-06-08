#include <pthread.h>
#include <iostream>
#include <string>
#include "mypthread.hpp"
#include <unistd.h>

void* pro_fun(void* args)
{
    sui::my_thread<int>* mt = static_cast<sui::my_thread<int>*>(args);
    int a = 0;
    int z = 10;
    while (z--)
    {
        std::cout << "pro_fun begin" << std::endl;
        mt->add(a);
        a++;
    }
    std::cout << "pro_fun end" << std::endl;
}
/*
void* pro_fun2(void* args)
{
    sui::my_thread<int>* mt = static_cast<sui::my_thread<int>*>(args);
    int a = 0;
    int z = 10;
    while(2)
    {
        std::cout << "pro_fun2 begin" << std::endl;
        mt->add(a);
        a++;
    }
    std::cout << "pro_fun2 end" << std::endl;
}*/

void* con_fun(void* args)
{

    sui::my_thread<int>* mt = static_cast<sui::my_thread<int>*>(args);
    int z = 10;
    while (z--)
    {
        int a = 0;
        mt->pop(&a);
        std::cout << "1a: " << a << std::endl;
        sleep(1);
    }
    std::cout << "con_fun end" << std::endl;
}
/*
void* con_fun2(void* args)
{

    sui::my_thread<int>* mt = static_cast<sui::my_thread<int>*>(args);
    int z = 10;
    while(2)
    {
        int a = 0;
        mt->pop(&a);
        std::cout << "2a: "<< a << std::endl;
        sleep(1);
    }
    std::cout << "con_fun2 end" << std::endl;
}*/

int main()
{

    sui::my_thread<int> my_data(5);

    pthread_t pro_thr, con_thr;
    if (pthread_create(&pro_thr, nullptr, pro_fun, &my_data))
    {
        std::cerr << "false to pro_fun create" << std::endl;
    }

    if (pthread_create(&con_thr, nullptr, con_fun, &my_data))
    {
        std::cerr << "false to con_fun create" << std::endl;
    }

    //pthread_create(&pro_thr, nullptr, pro_fun2, &my_data);
    //pthread_create(&con_thr, nullptr, con_fun2, &my_data);
    sleep(100);

    if (pthread_join(pro_thr, nullptr))
    {
        std::cerr << "false to pro_thr join" << std::endl;
    }
    std::cout << "pro_thr to join" << std::endl;

    if (pthread_join(con_thr, nullptr))
    {
        std::cerr << "false to con_thr join" << std::endl;
    }

    std::cout << "jie s" << std::endl;
    sleep(10);
    return 0;
}