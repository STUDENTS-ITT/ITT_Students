#include "vo/altitude.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>

#include "vo/util.hpp"

namespace vo {

namespace {

const char* const kAltAliases[] = {"alt", "altitude", "height", "hgt", "elevation",
                                   "ele", "agl", "asl", "z", "высота"};
const char* const kFrameAliases[] = {"frame", "frameid", "index", "idx", "sample",
                                     "time", "t", "timestamp", "sec", "кадр", "время"};

template <size_t N>
using Aliases = const char* const (&)[N];

std::optional<char> sniff_delimiter(const std::string& line) {
    for (char d : {',', ';', '\t', '|'})
        if (line.find(d) != std::string::npos) return d;
    return std::nullopt;   // пробелы
}

std::vector<std::string> split_cells(const std::string& line, std::optional<char> delim) {
    std::vector<std::string> cells;
    if (delim) {
        for (auto& c : split(line, *delim)) cells.push_back(trim(c));
    } else {
        std::istringstream ss(line);
        std::string c;
        while (ss >> c) cells.push_back(c);
    }
    return cells;
}

bool is_digits(const std::string& s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

std::optional<double> parse_double(const std::string& s) {
    const std::string t = trim(s);
    if (t.empty()) return std::nullopt;
    try {
        size_t pos = 0;
        const double v = std::stod(t, &pos);
        if (pos != t.size()) return std::nullopt;
        return v;
    } catch (...) {
        return std::nullopt;
    }
}

std::string strip_bom(std::string s) {
    if (s.size() >= 3 && static_cast<unsigned char>(s[0]) == 0xEF &&
        static_cast<unsigned char>(s[1]) == 0xBB && static_cast<unsigned char>(s[2]) == 0xBF)
        s.erase(0, 3);
    return s;
}

template <size_t N>
std::optional<size_t> pick_column(const std::vector<std::string>& header,
                                  const std::string& requested, const char* const (&aliases)[N],
                                  const std::string& what) {
    std::vector<std::string> low;
    for (const auto& h : header) low.push_back(lower(trim(strip_bom(h))));

    const std::string req = lower(trim(requested.empty() ? "auto" : requested));
    if (req != "auto") {
        if (is_digits(req)) return static_cast<size_t>(std::stoul(req));
        auto it = std::find(low.begin(), low.end(), req);
        if (it != low.end()) return static_cast<size_t>(it - low.begin());
        std::string joined;
        for (size_t i = 0; i < header.size(); ++i) joined += (i ? ", " : "") + header[i];
        throw std::runtime_error("Колонка \"" + requested + "\" (" + what +
                                 ") не найдена. Заголовок: " + joined);
    }
    for (const char* a : aliases) {
        auto it = std::find(low.begin(), low.end(), a);
        if (it != low.end()) return static_cast<size_t>(it - low.begin());
    }
    for (size_t i = 0; i < low.size(); ++i)
        for (const char* a : aliases)
            if (low[i].find(a) != std::string::npos) return i;
    return std::nullopt;
}

std::vector<double> read_altitude_file(const Config& cfg, size_t num_frames) {
    const std::string& path = cfg.altitude.file;
    if (path.empty() || !std::filesystem::exists(std::filesystem::u8path(path)))
        throw std::runtime_error("Файл высот не найден: " + path);

    std::ifstream in(std::filesystem::u8path(path), std::ios::binary);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!trim(line).empty()) lines.push_back(line);
    }
    if (lines.empty()) throw std::runtime_error("Файл высот пуст: " + path);
    lines[0] = strip_bom(lines[0]);

    const auto delim = sniff_delimiter(lines[0]);
    std::vector<std::string> header = split_cells(lines[0], delim);

    // Заголовок есть, если в первой строке встречаются буквы.
    bool has_header = false;
    for (const auto& c : header)
        for (unsigned char ch : c)
            if (std::isalpha(ch) || ch >= 0x80) has_header = true;
    const size_t first_row = has_header ? 1 : 0;
    if (!has_header)
        for (size_t i = 0; i < header.size(); ++i) header[i] = "col" + std::to_string(i);

    std::optional<size_t> i_alt = pick_column(header, cfg.altitude.column, kAltAliases, "высота");
    if (!i_alt) {
        if (header.size() == 1) i_alt = 0;
        else
            throw std::runtime_error("В файле " + path +
                                     " не найдена колонка высоты. Укажите её явно.");
    }
    std::optional<size_t> i_frame =
        pick_column(header, cfg.altitude.frame_column, kFrameAliases, "кадр/время");
    if (i_frame && *i_frame == *i_alt) i_frame.reset();

    std::vector<double> values, keys;
    std::vector<bool> key_ok;
    for (size_t r = first_row; r < lines.size(); ++r) {
        const auto cells = split_cells(lines[r], delim);
        if (*i_alt >= cells.size()) continue;
        auto v = parse_double(cells[*i_alt]);
        if (!v) continue;
        std::optional<double> k;
        if (i_frame && *i_frame < cells.size()) k = parse_double(cells[*i_frame]);
        values.push_back(*v);
        keys.push_back(k.value_or(0.0));
        key_ok.push_back(k.has_value());
    }
    if (values.empty())
        throw std::runtime_error("В колонке высоты файла " + path + " нет ни одного числа.");

    // Номера кадров 1-based, как в MATLAB/Python-версии.
    std::vector<double> targets(num_frames);
    for (size_t i = 0; i < num_frames; ++i) targets[i] = static_cast<double>(i) + 1.0;
    const double tmax = targets.empty() ? 0.0 : targets.back();
    const double tmin = targets.empty() ? 0.0 : targets.front();

    if (i_frame && std::all_of(key_ok.begin(), key_ok.end(), [](bool b) { return b; })) {
        std::vector<size_t> order(values.size());
        std::iota(order.begin(), order.end(), size_t{0});
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) { return keys[a] < keys[b]; });
        std::vector<double> ks(values.size()), vs(values.size());
        for (size_t i = 0; i < order.size(); ++i) {
            ks[i] = keys[order[i]];
            vs[i] = values[order[i]];
        }
        // Если ключ - время, а не номер кадра, приводим его к диапазону кадров.
        const double kmax = ks.back(), kmin = ks.front();
        if (kmax > 0 && kmax < 0.9 * tmax) {
            const double span = kmax > kmin ? kmax - kmin : 1.0;
            for (double& k : ks) k = tmin + (k - kmin) / span * (tmax - tmin);
        }
        return interp(targets, ks, vs);
    }

    if (values.size() == targets.size()) return values;    // по одному на кадр обработки
    if (static_cast<double>(values.size()) >= tmax) {       // по одному на кадр источника
        std::vector<double> out(num_frames);
        for (size_t i = 0; i < num_frames; ++i) out[i] = values[i];
        return out;
    }
    std::cout << "[ПРЕДУПРЕЖДЕНИЕ] В файле высот " << values.size() << " значений на "
              << static_cast<long long>(tmax) << " кадров - выборка растянута на весь диапазон.\n";
    std::vector<double> src(values.size());
    for (size_t i = 0; i < src.size(); ++i)
        src[i] = values.size() > 1
                     ? tmin + (tmax - tmin) * static_cast<double>(i) / (values.size() - 1)
                     : tmin;
    return interp(targets, src, values);
}

}  // namespace

std::vector<double> interp(const std::vector<double>& x, const std::vector<double>& xs,
                           const std::vector<double>& ys) {
    std::vector<double> out(x.size());
    if (xs.empty()) return out;
    for (size_t i = 0; i < x.size(); ++i) {
        const double v = x[i];
        if (v <= xs.front()) {
            out[i] = ys.front();
        } else if (v >= xs.back()) {
            out[i] = ys.back();
        } else {
            const size_t j = static_cast<size_t>(
                std::upper_bound(xs.begin(), xs.end(), v) - xs.begin());
            const double x0 = xs[j - 1], x1 = xs[j];
            const double t = x1 > x0 ? (v - x0) / (x1 - x0) : 0.0;
            out[i] = ys[j - 1] + t * (ys[j] - ys[j - 1]);
        }
    }
    return out;
}

std::string altitude_source(const Config& cfg) {
    const std::string src = lower(trim(cfg.altitude.source));
    if (src == "file" || src == "csv" || src == "txt") return "file";
    if (src == "const" || src == "constant" || src == "value" || src == "number" || src.empty())
        return "const";
    if (src == "scale" || src == "vo")
        throw std::runtime_error(
            "altitude.source = 'scale' ('vo') не поддерживается: высота из масштаба "
            "сцены накапливала дрейф в разы за полёт. Задайте высоту числом ('const') "
            "или по кадрам из файла ('file').");
    throw std::runtime_error("Неизвестное altitude.source = \"" + src +
                             "\". Допустимы 'const' и 'file'.");
}

std::vector<double> load_altitude_series(const Config& cfg, size_t num_frames) {
    const std::string source = altitude_source(cfg);
    if (source == "const") {
        std::cout << "Высота: постоянная, " << fmt("%.3f", cfg.altitude.value) << " м\n";
        return std::vector<double>(num_frames, cfg.altitude.value);
    }
    if (cfg.altitude.file.empty())
        throw std::runtime_error("Для altitude.source = 'file' задайте altitude.file.");
    std::vector<double> alt = read_altitude_file(cfg, num_frames);
    if (!alt.empty()) {
        const auto mm = std::minmax_element(alt.begin(), alt.end());
        std::cout << "Высота: из файла " << cfg.altitude.file << ", диапазон "
                  << fmt("%.2f..%.2f", *mm.first, *mm.second) << " м\n";
    }
    return alt;
}

}  // namespace vo
