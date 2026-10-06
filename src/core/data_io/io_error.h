// io_error.h — Диагностика ошибок открытия файлов.

#pragma once

#include <filesystem>
#include <iostream>
#include <string>

namespace data_io
{

// Вывод в stderr: путь к файлу и текущий рабочий каталог (для отладки).
inline void reportOpenError(const std::string &path)
{
    std::cerr << "error open: " << path << std::endl;
    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec)
    {
        std::cerr << "  cwd: " << cwd.string() << std::endl;
    }
}

} // namespace data_io
