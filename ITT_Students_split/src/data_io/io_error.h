#pragma once

// Единое сообщение о неудачном открытии файла.

#include <filesystem>
#include <iostream>
#include <string>

namespace data_io
{

// Кроме самого пути печатается рабочий каталог: обычная причина отказа —
// запуск не из того каталога, а по одному имени файла это не видно.
// Сообщения латиницей, чтобы не зависеть от кодовой страницы консоли.
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
