#include <iostream>
#include <string>
#include <utility>

template <typename T>
class MyClass {
private:
    T data;

public:
    // 默认构造函数
    MyClass() : data() {
        std::cout << "Default constructor called" << std::endl;
    }

    // 参数化构造函数
    explicit MyClass(const T& value) : data(value) {
        std::cout << "Parameterized constructor called" << std::endl;
    }

    // 拷贝构造函数
    MyClass(const MyClass& other) : data(other.data) {
        std::cout << "Copy constructor called" << std::endl;
    }

    // 移动构造函数
    MyClass(MyClass&& other) noexcept : data(std::move(other.data)) {
        std::cout << "Move constructor called" << std::endl;
    }

    // 拷贝赋值运算符
    MyClass& operator=(const MyClass& other) {
        std::cout << "Copy assignment operator called" << std::endl;
        if (this != &other) {
            data = other.data;
        }
        return *this;
    }

    // 移动赋值运算符
    MyClass& operator=(MyClass&& other) noexcept {
        std::cout << "Move assignment operator called" << std::endl;
        if (this != &other) {
            data = std::move(other.data);
        }
        return *this;
    }

    // 析构函数
    ~MyClass() {
        std::cout << "Destructor called" << std::endl;
    }

    // 显示数据
    void display() const {
        std::cout << "Data: " << data << std::endl;
    }
};
