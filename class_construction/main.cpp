#include <iostream>
#include <string>
#include <utility>

template <typename T>
class MyClass {
private:
    T data;

public:
    // Ĭ�Ϲ��캯��
    MyClass() : data() {
        std::cout << "Default constructor called" << std::endl;
    }

    // ���������캯��
    explicit MyClass(const T& value) : data(value) {
        std::cout << "Parameterized constructor called" << std::endl;
    }

    // �������캯��
    MyClass(const MyClass& other) : data(other.data) {
        std::cout << "Copy constructor called" << std::endl;
    }

    // �ƶ����캯��
    MyClass(MyClass&& other) noexcept : data(std::move(other.data)) {
        std::cout << "Move constructor called" << std::endl;
    }

    // ������ֵ�����
    MyClass& operator=(const MyClass& other) {
        std::cout << "Copy assignment operator called" << std::endl;
        if (this != &other) {
            data = other.data;
        }
        return *this;
    }

    // �ƶ���ֵ�����
    MyClass& operator=(MyClass&& other) noexcept {
        std::cout << "Move assignment operator called" << std::endl;
        if (this != &other) {
            data = std::move(other.data);
        }
        return *this;
    }

    // ��������
    ~MyClass() {
        std::cout << "Destructor called" << std::endl;
    }

    // ��ʾ����
    void display() const {
        std::cout << "Data: " << data << std::endl;
    }
};
