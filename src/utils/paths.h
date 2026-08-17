// paths.h — Поиск каталога с данными независимо от того, откуда запущена программа.
//
// Логика:
//   1. Проверяется рабочий каталог (запуск из корня проекта).
//   2. Проверяется каталог исполняемого файла (запуск из build/ или IDE).
//   Если imu.dat найден — возвращается путь к каталогу.

#pragma once

#include <filesystem>
#include <string>

namespace utils
{

// Каталог исполняемого файла по argv[0].
inline std::filesystem::path exeDir(const char *argv0)
{
    if (argv0 == nullptr || *argv0 == '\0')
    {
        return {};
    }
    std::error_code ec;
    const std::filesystem::path full = std::filesystem::absolute(argv0, ec);
    return ec ? std::filesystem::path{} : full.parent_path();
}

// Каталог данных: сначала рабочий каталог, затем каталог exe.
inline std::filesystem::path dataDir(const char *argv0, const std::string &probe)
{
    std::error_code ec;
    if (std::filesystem::exists(probe, ec))
    {
        return {};
    }
    const std::filesystem::path dir = exeDir(argv0);
    if (!dir.empty() && std::filesystem::exists(dir / probe, ec))
    {
        return dir;
    }
    return {};
}

} // namespace utils
