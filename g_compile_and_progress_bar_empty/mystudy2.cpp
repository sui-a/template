#include <iostream>
#include <cstring>
#include <unistd.h>
#include <iomanip> // 包含 std::setw 和 std::left

template <typename T>
void fun(T x, T y) {
    int z = static_cast<int>((static_cast<double>(x) / y) * 100);
    char w[100];
    char t[5] = "|/-\\"; // 修正数组大小为 5，包括 '\0'
    static int p = 0;
    memset(w, 0, sizeof(w));

    for (int i = 0; i < z && i < sizeof(w) - 1; i++) {
        w[i] = '#';
    }

    // 使用 std::cout 输出进度条
    std::cout << "[" << std::setw(100) << std::left << std::setfill(' ') << w
        << "][" << std::setw(3) << std::right << z << "%]["
        << t[p % 4] << "]";
    if (z >= 100)
        std::cout << "/n";
    else
        std::cout << "/r";
    std::cout.flush(); // 强制刷新输出缓冲区
    p++;
}

int main() {
    for (int i = 0; i <= 100; i++) {
        fun(i, 100);
        sleep(3); // 休眠 3 秒
    }
    std::cout << std::endl; // 最后换行
    return 0;
}