#include "vo/sequence.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

#include "vo/util.hpp"

namespace vo {

namespace fs = std::filesystem;

namespace {

const char* const kImageExt[] = {".png", ".jpg", ".jpeg", ".bmp", ".tif",
                                 ".tiff", ".pgm", ".ppm", ".pbm"};
const char* const kVideoExt[] = {".mp4", ".avi", ".mov", ".mkv",
                                 ".mpg", ".mpeg", ".wmv", ".m4v"};
const char* const kUrlSchemes[] = {"http://", "https://", "rtsp://", "rtmp://",
                                   "rtmps://", "udp://", "tcp://", "file://",
                                   "ftp://"};

bool has_ext(const std::string& path, const char* const* list, size_t n) {
    const std::string ext = lower(fs::u8path(path).extension().u8string());
    for (size_t i = 0; i < n; ++i)
        if (ext == list[i]) return true;
    return false;
}

bool is_url(const std::string& s) {
    const std::string l = lower(s);
    for (const char* sch : kUrlSchemes)
        if (starts_with(l, sch)) return true;
    return false;
}

std::string normalize_source(const std::string& source) {
    if (starts_with(lower(source), "file://")) {
        std::string text = source.substr(7);
        while (!text.empty() && text[0] == '/') text.erase(0, 1);
        if (text.size() > 1 && text[1] == ':') return text;   // C:/... на Windows
        return "/" + text;
    }
    return source;
}

// Простое сопоставление с маской: '*' - любая подстрока, '?' - один символ.
bool wildcard_match(const std::string& pat, const std::string& s) {
    size_t p = 0, i = 0, star = std::string::npos, mark = 0;
    while (i < s.size()) {
        if (p < pat.size() && (pat[p] == '?' || pat[p] == s[i])) {
            ++p;
            ++i;
        } else if (p < pat.size() && pat[p] == '*') {
            star = p++;
            mark = i;
        } else if (star != std::string::npos) {
            p = star + 1;
            i = ++mark;
        } else {
            return false;
        }
    }
    while (p < pat.size() && pat[p] == '*') ++p;
    return p == pat.size();
}

}  // namespace

std::vector<std::string> list_images(const std::string& folder) {
    std::vector<std::string> files;
    std::error_code ec;
    const bool is_mask = folder.find_first_of("*?") != std::string::npos;
    if (is_mask) {
        const fs::path p = fs::u8path(folder);
        const fs::path dir = p.parent_path().empty() ? fs::path(".") : p.parent_path();
        const std::string pat = p.filename().u8string();
        if (fs::is_directory(dir, ec))
            for (const auto& e : fs::directory_iterator(dir, ec))
                if (e.is_regular_file() && wildcard_match(pat, e.path().filename().u8string()))
                    files.push_back(e.path().u8string());
    } else {
        if (!fs::is_directory(fs::u8path(folder), ec))
            throw std::runtime_error("Папка с изображениями не найдена: " + folder);
        for (const auto& e : fs::directory_iterator(fs::u8path(folder), ec))
            if (e.is_regular_file() &&
                has_ext(e.path().u8string(), kImageExt, sizeof(kImageExt) / sizeof(*kImageExt)))
                files.push_back(e.path().u8string());
    }
    if (files.empty())
        throw std::runtime_error(
            "В " + folder +
            " нет изображений. Поместите туда последовательность кадров с именами, "
            "сохраняющими правильный порядок, например 000000.png, 000001.png, ...");
    std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
        return lower(fs::u8path(a).filename().u8string()) <
               lower(fs::u8path(b).filename().u8string());
    });
    return files;
}

ImageFolderSequence::ImageFolderSequence(const std::string& folder)
    : files_(list_images(folder)) {}

cv::Mat ImageFolderSequence::read(size_t index) {
    cv::Mat img = cv::imread(files_.at(index), cv::IMREAD_UNCHANGED);
    if (img.empty()) throw std::runtime_error("Не удалось прочитать кадр: " + files_.at(index));
    return img;
}

VideoSequence::VideoSequence(const std::string& path, long long max_frames, bool fast_skip)
    : source_(path), fast_skip_(fast_skip) {
    is_stream_ = is_url(path);
    if (!is_stream_ && !fs::exists(fs::u8path(path)))
        throw std::runtime_error("Видеофайл не найден: " + path);
    if (is_stream_) std::cout << "Открываю поток (может занять несколько секунд)...\n";

    cap_.open(path);
    if (!cap_.isOpened())
        throw std::runtime_error(
            "Не удалось открыть видео: " + path +
            (is_stream_ ? "  Для сетевых источников OpenCV нужен backend FFmpeg и "
                          "ПРЯМАЯ ссылка на видеофайл или поток."
                        : ""));
    fps_ = cap_.get(cv::CAP_PROP_FPS);
    if (!(fps_ > 0)) fps_ = 0.0;
    count_ = static_cast<long long>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
    if (count_ <= 0) {
        if (max_frames < 0)
            throw std::runtime_error(
                "Не удалось определить число кадров в источнике. Для потоков и "
                "ссылок задайте предел: --max-frames N. Либо скачайте видео и "
                "укажите путь к файлу.");
        count_ = max_frames;
        length_known_ = false;
        std::cout << "Число кадров недоступно, ограничиваюсь " << count_
                  << " кадрами (перемотка назад невозможна)\n";
    }
}

std::string VideoSequence::name(size_t index) const {
    std::string base = source_.substr(0, source_.find('?'));
    std::string file = fs::u8path(base).filename().u8string();
    if (file.empty()) file = "stream";
    return fmt("%s#%06zu", file.c_str(), index);
}

cv::Mat VideoSequence::read(size_t index) {
    const long long idx = static_cast<long long>(index);
    if (idx < pos_) {
        if (!length_known_) return cv::Mat();   // поток назад не мотается
        cap_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(idx));
        pos_ = idx;
    }
    cv::Mat frame;
    while (pos_ < idx) {
        // Промежуточные кадры нужны только чтобы добраться до нужного:
        // grab() декодирует их, но не разворачивает в BGR.
        const bool ok = fast_skip_ ? cap_.grab() : cap_.read(frame);
        ++pos_;
        if (!ok) return cv::Mat();
    }
    if (!cap_.read(frame)) {
        ++pos_;
        return cv::Mat();
    }
    ++pos_;
    return frame;
}

std::unique_ptr<Sequence> open_sequence(const std::string& source_in, const Config& cfg) {
    const std::string source = normalize_source(source_in);
    const std::string no_query = source.substr(0, source.find('?'));
    const bool video_ext = has_ext(no_query, kVideoExt, sizeof(kVideoExt) / sizeof(*kVideoExt));

    if (is_url(source) || (video_ext && fs::is_regular_file(fs::u8path(source)))) {
        auto seq = std::make_unique<VideoSequence>(source, cfg.max_frames, cfg.perf.fast_skip);
        std::cout << "Источник: видео " << source << ", кадров"
                  << (seq->length_known() ? "" : " (предел)") << " " << seq->size() << ", "
                  << fmt("%.2f", seq->fps()) << " fps\n";
        return seq;
    }
    // Расширение видеофайла, но файла нет: иначе пользователь получил бы
    // сообщение про ненайденную ПАПКУ на месте ненайденного файла.
    if (video_ext) throw std::runtime_error("Видеофайл не найден: " + source);

    auto seq = std::make_unique<ImageFolderSequence>(source);
    std::cout << "Источник: изображения " << source << ", кадров " << seq->size() << "\n";
    return seq;
}

}  // namespace vo
