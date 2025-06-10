#include "log.hpp"
#include <Windows.h>


int main()
{
	DWORD bufferSize = MAX_PATH;
	std::vector<TCHAR> buffer(bufferSize);

	if (GetCurrentDirectory(bufferSize, buffer.data()) != 0) {
		std::wcout << L"Current Working Directory: " << std::wstring(buffer.begin(), buffer.end()) << std::endl;
	}

	LOG(sui::My_level::debug) << "hello world" << "zz " << "tt";
	Sleep(20000);
	
	return 0;
}