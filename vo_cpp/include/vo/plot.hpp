// Минимальная отрисовка графиков средствами OpenCV (замена matplotlib).
// Шрифты OpenCV (Hershey) знают только ASCII, поэтому подписи - латиницей.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vo {

// Палитра tab10 (BGR).
cv::Scalar tab_color(int index);
extern const cv::Scalar kBlack, kGray;

struct Series {
    std::vector<double> x, y;
    cv::Scalar color{0, 0, 0};
    std::string label;
    double width = 1.5;
    enum class Style { Line, Dots, Marker } style = Style::Line;
};

struct VLine {
    double x = 0;
    std::string label;
    cv::Scalar color{128, 128, 128};
};

struct Span {
    double x0 = 0, x1 = 0;
    cv::Scalar color{230, 160, 90};
    double alpha = 0.15;
};

struct Axes {
    std::string title, xlabel, ylabel;
    std::vector<Series> series;
    std::vector<VLine> vlines;
    std::vector<Span> spans;
    bool equal = false;                 // одинаковый масштаб по осям
    bool legend = true;
    bool legend_upper_right = false;    // иначе нижний левый угол
    std::optional<std::array<double, 4>> limits;   // xmin, xmax, ymin, ymax
    bool hide = false;                  // пустая ячейка
};

// Сетка rows x cols осей на холсте заданного размера.
cv::Mat render_figure(const std::vector<Axes>& axes, int rows, int cols, const cv::Size& size,
                      const std::string& suptitle = "");

// Сохраняет PNG; создаёт каталог. false, если запись не удалась.
bool save_figure(const std::string& path, const cv::Mat& image);

}  // namespace vo
