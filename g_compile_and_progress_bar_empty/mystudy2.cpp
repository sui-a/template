#include <iostream>
#include <cstring>
#include <unistd.h>
#include <iomanip> // ���� std::setw �� std::left

template <typename T>
void fun(T x, T y) {
    int z = static_cast<int>((static_cast<double>(x) / y) * 100);
    char w[100];
    char t[5] = "|/-\\"; // ���������СΪ 5������ '\0'
    static int p = 0;
    memset(w, 0, sizeof(w));

    for (int i = 0; i < z && i < sizeof(w) - 1; i++) {
        w[i] = '#';
    }

    // ʹ�� std::cout ���������
    std::cout << "[" << std::setw(100) << std::left << std::setfill(' ') << w
        << "][" << std::setw(3) << std::right << z << "%]["
        << t[p % 4] << "]";
    if (z >= 100)
        std::cout << "/n";
    else
        std::cout << "/r";
    std::cout.flush(); // ǿ��ˢ�����������
    p++;
}

int main() {
    for (int i = 0; i <= 100; i++) {
        fun(i, 100);
        sleep(3); // ���� 3 ��
    }
    std::cout << std::endl; // �����
    return 0;
}