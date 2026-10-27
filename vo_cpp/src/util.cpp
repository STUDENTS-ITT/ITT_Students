#include "vo/util.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <io.h>
#  include <windows.h>
#  define VO_ISATTY _isatty
#  define VO_FILENO _fileno
#else
#  include <unistd.h>
#  define VO_ISATTY isatty
#  define VO_FILENO fileno
#endif

namespace vo {

void init_console() {
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
}

std::string fmt(const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(nullptr, 0, format, copy);
    va_end(copy);
    std::string out;
    if (n > 0) {
        out.resize(static_cast<size_t>(n) + 1);
        std::vsnprintf(out.data(), out.size(), format, args);
        out.resize(static_cast<size_t>(n));
    }
    va_end(args);
    return out;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == delim) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

bool starts_with(const std::string& s, const std::string& prefix) {
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with_ci(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    return lower(s.substr(s.size() - suffix.size())) == lower(suffix);
}

std::string safe_file_name(const std::string& text) {
    std::string out = text;
    for (char& c : out) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (!(std::isalnum(u) && u < 128) && c != '_' && c != '-') c = '_';
    }
    return out;
}

double safe_mean(const std::vector<double>& values) {
    double sum = 0.0;
    size_t n = 0;
    for (double v : values) {
        if (std::isfinite(v)) {
            sum += v;
            ++n;
        }
    }
    return n ? sum / static_cast<double>(n) : std::nan("");
}

double median(std::vector<double> v) {
    if (v.empty()) return std::nan("");
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(mid), v.end());
    const double hi = v[mid];
    if (v.size() % 2) return hi;
    const double lo = *std::max_element(v.begin(), v.begin() + static_cast<long>(mid));
    return 0.5 * (lo + hi);
}

void make_dirs(const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::u8path(path), ec);
}

std::string parent_path(const std::string& path) {
    return std::filesystem::u8path(path).parent_path().u8string();
}

std::string fmt_g15(double v) {
    if (!std::isfinite(v)) return "NaN";
    return fmt("%.15g", v);
}

void CsvTable::write(const std::string& path) const {
    make_dirs(parent_path(path));
    std::ofstream out(std::filesystem::u8path(path), std::ios::binary);
    auto join = [&](const std::vector<std::string>& cells) {
        for (size_t i = 0; i < cells.size(); ++i) {
            if (i) out << ',';
            const std::string& c = cells[i];
            if (c.find_first_of(",\"\n") != std::string::npos) {
                out << '"';
                for (char ch : c) {
                    if (ch == '"') out << '"';
                    out << ch;
                }
                out << '"';
            } else {
                out << c;
            }
        }
        out << '\n';
    };
    join(columns);
    for (const auto& r : rows) join(r);
}

void CsvTable::print(std::ostream& os) const {
    if (rows.empty()) {
        os << "(пусто)\n";
        return;
    }
    std::vector<size_t> widths(columns.size());
    auto count_chars = [](const std::string& s) {
        size_t n = 0;
        for (unsigned char c : s) n += ((c & 0xC0) != 0x80);
        return n;
    };
    for (size_t i = 0; i < columns.size(); ++i) {
        widths[i] = count_chars(columns[i]);
        for (const auto& r : rows)
            if (i < r.size()) widths[i] = std::max(widths[i], count_chars(r[i]));
    }
    auto line = [&](const std::vector<std::string>& cells) {
        for (size_t i = 0; i < columns.size(); ++i) {
            const std::string c = i < cells.size() ? cells[i] : "";
            os << c << std::string(widths[i] - count_chars(c), ' ');
            if (i + 1 < columns.size()) os << "  ";
        }
        os << '\n';
    };
    line(columns);
    std::vector<std::string> dashes;
    for (size_t w : widths) dashes.emplace_back(w, '-');
    line(dashes);
    for (const auto& r : rows) line(r);
}

// ---------------------------------------------------------------- ConsoleLog

namespace {
std::streambuf* real_cout_buf() {
    static std::streambuf* buf = std::cout.rdbuf();
    return buf;
}
}  // namespace

struct ConsoleLog::Impl : public std::streambuf {
    std::streambuf* orig = nullptr;     // куда писали до нас
    std::string pending_path;
    std::string memory;                 // пока файл неизвестен
    std::ofstream file;
    bool file_open = false;
    bool progress_active = false;
    size_t progress_len = 0;
    bool tty = false;

    void write_out(const char* s, std::streamsize n) {
        if (orig) orig->sputn(s, n);
        if (file_open) file.write(s, n);
        else memory.append(s, static_cast<size_t>(n));
    }

    void close_progress() {
        if (progress_active && orig) {
            orig->sputc('\n');
            orig->pubsync();
        }
        progress_active = false;
        progress_len = 0;
    }

protected:
    std::streamsize xsputn(const char* s, std::streamsize n) override {
        if (progress_active) {
            bool visible = false;
            for (std::streamsize i = 0; i < n; ++i)
                if (!std::isspace(static_cast<unsigned char>(s[i]))) { visible = true; break; }
            if (visible) close_progress();
        }
        write_out(s, n);
        return n;
    }
    int_type overflow(int_type ch) override {
        if (ch == traits_type::eof()) return traits_type::not_eof(ch);
        const char c = static_cast<char>(ch);
        xsputn(&c, 1);
        return ch;
    }
    int sync() override {
        if (orig) orig->pubsync();
        if (file_open) file.flush();
        return 0;
    }
};

ConsoleLog::ConsoleLog(const std::string& path) : impl_(new Impl) {
    impl_->orig = std::cout.rdbuf(impl_.get());
    impl_->tty = (impl_->orig == real_cout_buf()) && VO_ISATTY(VO_FILENO(stdout));
    if (!path.empty()) set_path(path);
}

ConsoleLog::~ConsoleLog() {
    end_progress();
    std::cout.flush();
    std::cout.rdbuf(impl_->orig);
    if (impl_->file_open) impl_->file.close();
}

void ConsoleLog::set_path(const std::string& path) {
    make_dirs(parent_path(path));
    impl_->file.open(std::filesystem::u8path(path), std::ios::binary);
    if (!impl_->file) return;
    impl_->file_open = true;
    impl_->file << impl_->memory;
    impl_->memory.clear();
    impl_->memory.shrink_to_fit();
}

bool ConsoleLog::progress(const std::string& text) {
    if (!impl_->tty) return false;
    std::streambuf* out = impl_->orig;
    const size_t pad = impl_->progress_len > text.size() ? impl_->progress_len - text.size() : 0;
    const std::string line = "\r" + text + std::string(pad, ' ');
    out->sputn(line.data(), static_cast<std::streamsize>(line.size()));
    out->pubsync();
    impl_->progress_len = text.size();
    impl_->progress_active = true;
    return true;
}

void ConsoleLog::end_progress() { impl_->close_progress(); }

}  // namespace vo
