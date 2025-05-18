#include <iostream>
#include <signal.h>
#include <unistd.h>

void fun(int sig)
{
    std::cout << "signal:" << sig << " success" << std::endl;
    signal(SIGINT, SIG_DFL);
}

void printpending(const sigset_t& sig)
{
    std::cout << "process: " << getpid() << "  ";
    for (int i = 31; i >= 1; i--)
    {
        if (sigismember(&sig, i))
        {
            std::cout << "1";
        }
        else
        {
            std::cout << "0";
        }
    }
    std::cout << std::endl;
}

int main()
{
    //set signal capture
    signal(SIGINT, fun);

    //signal mask set
    sigset_t one, two;
    sigemptyset(&one);
    sigemptyset(&two);

    //be controlled in signal two
    sigaddset(&one, SIGINT);

    int n = sigprocmask(SIG_SETMASK, &one, &two);
    //you do not need to judge, because normal will succeed;
    if (n != 0)
    {
        std::cerr << "sigprocmask error: " << errno << std::endl;
    }

    int i = 0;

    while (true)
    {
        //obtain pending signal
        sigset_t pending;
        int m = sigpending(&pending);
        if (m != 0)
        {
            std::cerr << "sigpending error: " << errno << std::endl;
        }

        printpending(pending);

        sleep(1);
        if (i == 10)
        {
            int n = sigprocmask(SIG_SETMASK, &two, nullptr);
        }
        i++;
    }
    return 0;
}#include <iostream>
#include <signal.h>
#include <unistd.h>

void fun(int sig)
{
    std::cout << "signal:" << sig << " success" << std::endl;
    signal(SIGINT, SIG_DFL);
}

void printpending(const sigset_t& sig)
{
    std::cout << "process: " << getpid() << "  ";
    for (int i = 31; i >= 1; i--)
    {
        if (sigismember(&sig, i))
        {
            std::cout << "1";
        }
        else
        {
            std::cout << "0";
        }
    }
    std::cout << std::endl;
}

int main()
{
    //set signal capture
    signal(SIGINT, fun);

    //signal mask set
    sigset_t one, two;
    sigemptyset(&one);
    sigemptyset(&two);

    //be controlled in signal two
    sigaddset(&one, SIGINT);

    int n = sigprocmask(SIG_SETMASK, &one, &two);
    //you do not need to judge, because normal will succeed;
    if (n != 0)
    {
        std::cerr << "sigprocmask error: " << errno << std::endl;
    }

    int i = 0;

    while (true)
    {
        //obtain pending signal
        sigset_t pending;
        int m = sigpending(&pending);
        if (m != 0)
        {
            std::cerr << "sigpending error: " << errno << std::endl;
        }

        printpending(pending);

        sleep(1);
        if (i == 10)
        {
            int n = sigprocmask(SIG_SETMASK, &two, nullptr);
        }
        i++;
    }
    return 0;
}