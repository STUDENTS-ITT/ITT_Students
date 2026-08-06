#pragma once

// Поиск каталога с данными независимо от того, откуда запущена программа.

#include <filesystem>
#include <string>

namespace utils
{

// Каталог исполняемого файла по argv[0]; пустой путь, если определить не удалось.
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

// Каталог, из которого читаются imu.dat, gps.dat, angle.dat и куда пишутся
// результаты. Сначала проверяется рабочий каталог, затем каталог exe: запуск
// из VS Code с рабочим каталогом src даёт тот же результат, что и из корня
// проекта. Пустой путь означает рабочий каталог.
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
