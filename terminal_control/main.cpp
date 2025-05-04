#include <iostream>
#include <windows.h> // Windows ϵͳ����

// �����ն˵�ԭʼ״̬
#ifdef _WIN32
HANDLE hStdin;
DWORD fdwSaveOldMode;
#else
termios original_termios;
#endif

void saveTerminalState() {
    hStdin = GetStdHandle(STD_INPUT_HANDLE);
    GetConsoleMode(hStdin, &fdwSaveOldMode); // ���浱ǰ�ն�ģʽ
}

void restoreTerminalState() {
    SetConsoleMode(hStdin, fdwSaveOldMode); // �ָ��ն�ģʽ
}

void clearScreen() {
    system("cls"); // �� Windows ������
}

int main() {
    // �����ն�״̬
    saveTerminalState();

    // ע��������ʱ��������
    std::atexit(restoreTerminalState);

    // �����Ļ
    clearScreen();

    // ����Զ�������
    std::cout << "Hello, this is a custom message!" << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;

    // �ȴ��û�����
    std::cin.get();

    // �������ʱ��restoreTerminalState �ᱻ�Զ�����
    return 0;
}