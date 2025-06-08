#pragma once
#include <iostream>
#include <pthread.h>
#include <queue>


namespace sui
{
    class my_mutex
    {
    public:
        explicit my_mutex(pthread_mutex_t& mutex)
            :_mutex(mutex)
        {
            int a_lock_back = pthread_mutex_lock(&_mutex);
            if (a_lock_back != 0)
            {
                throw std::runtime_error("Failed to lock mutex");
            }
        }

        my_mutex(const my_mutex&) = delete;
        my_mutex& operator=(const my_mutex&) = delete;

        ~my_mutex()
        {
            pthread_mutex_unlock(&_mutex);
        }
    private:
        pthread_mutex_t& _mutex;
    };

    template<class T>
    class my_thread
    {
    public:
        my_thread(int max)
            :_max(max),
            _current(0),
            _pop_wait_count(0),
            _add_wait_count(0)
        {
            //chu shi hua hu chi suo
            pthread_mutex_init(&_mut, nullptr);

            //chu shi hua tiao jian bian liang 
            pthread_cond_init(&_cond_add, nullptr);
            pthread_cond_init(&_cond_pop, nullptr);
        }

        void add(T& data)
        {
            std::cout << "add begin" << std::endl;

            //add mut
            my_mutex my_mutex_add(_mut);

            //shi yong xun huan duo ci pan duan, fang zhi wei ji huo
            while (_current >= _max)
            {
                _add_wait_count++;
                std::cout << "add wait" << std::endl;
                pthread_cond_wait(&_cond_add, &_mut);
                _add_wait_count--;
            }

            _data.push(data);
            _current++;

            if (_pop_wait_count > 0)
            {
                pthread_cond_broadcast(&_cond_pop);
            }
            else if (_add_wait_count > 0)
            {
                pthread_cond_broadcast(&_cond_add);
            }
        }

        void pop(T* data)
        {
            std::cout << "pop begin" << std::endl;
            my_mutex my_mutex_pop(_mut);

            while (_current == 0)
            {
                _pop_wait_count++;
                std::cout << "pop wait" << std::endl;
                pthread_cond_wait(&_cond_pop, &_mut);
                _pop_wait_count--;
            }

            *data = _data.front();
            _data.pop();
            _current--;


            if (_add_wait_count > 0)
            {
                pthread_cond_broadcast(&_cond_add);
            }
            else if (_pop_wait_count > 0)
            {
                pthread_cond_broadcast(&_cond_pop);
            }
        }

        ~my_thread()
        {
            std::cout << " xi gou begin" << std::endl;
            pthread_mutex_destroy(&_mut);
            pthread_cond_destroy(&_cond_add);
            pthread_cond_destroy(&_cond_pop);
        }


    private:
        std::queue<T> _data;
        int _max;
        int _current;

        //hu chi suo
        pthread_mutex_t _mut;

        //tiao jian bian liang 
        pthread_cond_t _cond_add;  //shen can tiao jian
        pthread_cond_t _cond_pop;  //xiao fei tiao jian

        //deng dai diao du d shu liang 
        int _add_wait_count;
        int _pop_wait_count;
    };
}