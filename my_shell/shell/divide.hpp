#pragma once
#include <windows.h>
#include <iostream>
#include <string>
#include <filesystem>
#include <Lmcons.h>
#include <queue>

//根据空格进行切割
std::queue<std::string> string_space_divide(const std::string& arr);
