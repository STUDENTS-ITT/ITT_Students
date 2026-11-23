// Источники кадров: папка/маска с картинками, видеофайл, прямая ссылка на поток.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "vo/config.hpp"

namespace vo {

class Sequence {
public:
    virtual ~Sequence() = default;
    virtual size_t size() const = 0;
    virtual std::string name(size_t index) const = 0;
    // Пустой cv::Mat, если источник закончился раньше.
    virtual cv::Mat read(size_t index) = 0;
    virtual double fps() const { return 0.0; }
    virtual bool length_known() const { return true; }
    virtual bool is_video() const { return false; }
};

class ImageFolderSequence : public Sequence {
public:
    explicit ImageFolderSequence(const std::string& folder_or_mask);
    size_t size() const override { return files_.size(); }
    std::string name(size_t index) const override { return files_.at(index); }
    cv::Mat read(size_t index) override;

private:
    std::vector<std::string> files_;
};

class VideoSequence : public Sequence {
public:
    VideoSequence(const std::string& path, long long max_frames, bool fast_skip);
    size_t size() const override { return static_cast<size_t>(count_); }
    std::string name(size_t index) const override;
    cv::Mat read(size_t index) override;
    double fps() const override { return fps_; }
    bool length_known() const override { return length_known_; }
    bool is_video() const override { return true; }

private:
    cv::VideoCapture cap_;
    std::string source_;
    bool fast_skip_ = true;
    bool is_stream_ = false;
    bool length_known_ = true;
    double fps_ = 0.0;
    long long count_ = 0;
    long long pos_ = 0;
};

// Список картинок: сортировка по имени файла в нижнем регистре, как в Python.
std::vector<std::string> list_images(const std::string& folder_or_mask);

// Открывает папку, маску "dir/*.png", видеофайл или URL.
std::unique_ptr<Sequence> open_sequence(const std::string& source, const Config& cfg);

}  // namespace vo
