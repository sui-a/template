#include <iostream>
#include <windows.h> // Windows 系统调用

// 保存终端的原始状态
#ifdef _WIN32
HANDLE hStdin;
DWORD fdwSaveOldMode;
#else
termios original_termios;
#endif

void saveTerminalState() {
    hStdin = GetStdHandle(STD_INPUT_HANDLE);
    GetConsoleMode(hStdin, &fdwSaveOldMode); // 保存当前终端模式
}

void restoreTerminalState() {
    SetConsoleMode(hStdin, fdwSaveOldMode); // 恢复终端模式
}

void clearScreen() {
    system("cls"); // 在 Windows 上清屏
}

int main() {
    // 保存终端状态
    saveTerminalState();

    // 注册程序结束时的清理函数
    std::atexit(restoreTerminalState);

    // 清空屏幕
    clearScreen();

    // 输出自定义内容
    std::cout << "Hello, this is a custom message!" << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;

    // 等待用户输入
    std::cin.get();

    // 程序结束时，restoreTerminalState 会被自动调用
    return 0;
}