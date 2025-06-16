#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include "Log.hpp"
#include "Thread.hpp"
#include "Cond.hpp"
#include "Mutex.hpp"

// .hpp header only

namespace ThreadPoolModule
{
    using namespace ThreadModlue;
    using namespace LogModule;
    using namespace CondModule;
    using namespace MutexModule;

    static const int gnum = 5;
    template <typename T>
    class ThreadPool
    {
    private:
        void WakeUpAllThread()
        {
            LockGuard lockguard(_mutex);
            if (_sleepernum)
                _cond.Broadcast();
            LOG(LogLevel::INFO) << "唤醒所有的休眠线程";
        }

        void WakeUpOne()
        {
            _cond.Signal();
            LOG(LogLevel::INFO) << "唤醒一个休眠线程";
        }

        ThreadPool(int num = gnum) : _num(num), _isrunning(false), _sleepernum(0)
        {
            for (int i = 0; i < num; i++)
            {
                _threads.emplace_back(
                    [this]()
                    {
                        HandlerTask();
                    });
            }
        }
        void Start()
        {
            if (_isrunning)
                return;
            _isrunning = true;
            for (auto& thread : _threads)
            {
                thread.Start();
                // LOG(LogLevel::INFO) << "start new thread success: " << thread.Name();
            }
        }

        ThreadPool(const ThreadPool<T>&) = delete;
        ThreadPool<T>& operator=(const ThreadPool<T>&) = delete;

    public:
        static ThreadPool<T>* GetInstance()
        {
            if (inc == nullptr)
            {
                LockGuard lockguard(_lock);

                LOG(LogLevel::DEBUG) << "获取单例....";
                if (inc == nullptr)
                {
                    LOG(LogLevel::DEBUG) << "首次使用单例, 创建之....";
                    inc = new ThreadPool<T>();
                    inc->Start();
                }
            }

            return inc;
        }
        void Stop()
        {
            if (!_isrunning)
                return;
            _isrunning = false;

            // 唤醒所有的线层
            WakeUpAllThread();
        }
        void Join()
        {
            for (auto& thread : _threads)
            {
                thread.Join();
            }
        }
        void HandlerTask()
        {
            char name[128];
            pthread_getname_np(pthread_self(), name, sizeof(name));
            while (true)
            {
                T t;
                {
                    LockGuard lockguard(_mutex);
                    // 1. a.队列为空 b. 线程池没有退出
                    while (_taskq.empty() && _isrunning)
                    {
                        _sleepernum++;
                        _cond.Wait(_mutex);
                        _sleepernum--;
                    }
                    // 2. 内部的线程被唤醒
                    if (!_isrunning && _taskq.empty())
                    {
                        LOG(LogLevel::INFO) << name << " 退出了, 线程池退出&&任务队列为空";
                        break;
                    }

                    // 一定有任务
                    t = _taskq.front(); // 从q中获取任务，任务已经是线程私有的了！！！
                    _taskq.pop();
                }
                t(); // 处理任务，需/要在临界区内部处理吗？1 0
            }
        }
        bool Enqueue(const T& in)
        {
            if (_isrunning)
            {
                LockGuard lockguard(_mutex);
                _taskq.push(in);
                if (_threads.size() == _sleepernum)
                    WakeUpOne();
                return true;
            }
            return false;
        }
        ~ThreadPool()
        {
        }

    private:
        std::vector<Thread> _threads;
        int _num; // 线程池中，线程的个数
        std::queue<T> _taskq;
        Cond _cond;
        Mutex _mutex;

        bool _isrunning;
        int _sleepernum;

        // bug??
        static ThreadPool<T>* inc; // 单例指针
        static Mutex _lock;
    };

    template <typename T>
    ThreadPool<T>* ThreadPool<T>::inc = nullptr;

    template <typename T>
    Mutex ThreadPool<T>::_lock;

}