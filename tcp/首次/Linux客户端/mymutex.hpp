#pragma once
#include <pthread.h>


namespace sui
{
    class my_mutex
    {
    public:
        my_mutex()
        {
            pthread_mutex_init(&_mut, nullptr);
        }

        bool lock()
        {
            int n = pthread_mutex_lock(&_mut);
            if (n)
                return false;
            return true;
        }

        bool unlock()
        {
            int n = pthread_mutex_unlock(&_mut);
            if (n)
                return false;
            return true;
        }

        pthread_mutex_t* get()
        {
            return &_mut;
        }

        ~my_mutex()
        {
            pthread_mutex_destroy(&_mut);
        }

    private:
        pthread_mutex_t _mut;
    };
}