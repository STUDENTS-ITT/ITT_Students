// vo_final - монокулярная визуальная одометрия по аэросъёмке (C++-порт vo_final2.py).
//
//   vo_final --input frames/ --calibration camera.json --altitude-file height.csv
//   vo_final --input video.mp4 --altitude 100 --motion-model homography
//   vo_final --input frames/ --mode trajectories
//   vo_final --list
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

#include "vo/altitude.hpp"
#include "vo/config.hpp"
#include "vo/features.hpp"
#include "vo/pipeline.hpp"
#include "vo/util.hpp"

namespace {

void print_help() {
    std::cout <<
R"(Монокулярная визуальная одометрия по аэросъёмке (C++-порт vo_final2.py).

Использование: vo_final [ключи]

Источник и результаты
  -i, --input PATH          папка с кадрами, маска "dir/*.png", видеофайл или ссылка
      --out-dir DIR         папка для результатов (по умолчанию ./results)
      --calibration FILE    калибровка камеры (.json/.yml/.xml)
      --altitude M          высота съёмки числом, м
      --altitude-file FILE  высота по кадрам (CSV)
      --max-frames N        ограничить число кадров
Режим
  -m, --mode vo|trajectories   одна траектория (по умолчанию) или каждой связкой
  -a, --algorithm LIST         (игнорируется; поддерживается только FAST-FREAK)
      --list                   показать информацию и выйти
Модель движения и кадры
      --motion-model M         (игнорируется; поддерживается только homography)
      --lk MODE                трекинг Лукаса-Канаде: rotation (на поворотах) | always | only (только LK, без FREAK) | off
      --no-shadow              не маскировать собственную тень и шасси БПЛА
      --frame-step N           шаг по кадрам (используется с --fixed-step)
      --fixed-step N           выключить адаптивный выбор кадров, шаг N
      --target-shift PCT       цель адаптивного выбора: смещение сцены, % ширины
      --scale-correction K     поправочный множитель масштаба
      --scale V                translation_scale - длина шага без метрики
      --preset P               accurate | balanced | realtime | fast
Обрезка на поворотах
      --rotation-crop / --no-rotation-crop
      --rotation-threshold DPS порог включения режима поворота, град/с
      --rotation-cut F         одинаковая обрезка со всех сторон, доля 0..0.45
Экспорт GPS
      --fps F  --ref-lat X  --ref-lon X  --export-scale X
Экспорт для INS/Kalman
      --export-ins             экспорт в gps.dat и angle.dat для add-kalman-report
      --ins-output-dir DIR     каталог для файлов INS (по умолчанию ins_output)
      --ins-fps F              частота кадров для timestamps INS (по умолчанию из --fps)
Вывод и скорость
      --show-video / --no-show-video   окно с видеопотоком (q/Esc - выход, пробел - пауза)
      --live-plot                      траектория в окне по ходу работы
      --profile                        время по этапам
      --quiet                          без строки на каждый кадр
      --no-figures                     не сохранять картинки соответствий
      --threads N                      потоков OpenCV (0 = все ядра)
  -h, --help
)";
}

struct Args {
    std::map<std::string, std::string> values;
    std::vector<std::string> flags;
    bool has(const std::string& k) const { return values.count(k) > 0; }
    bool flag(const std::string& k) const {
        return std::find(flags.begin(), flags.end(), k) != flags.end();
    }
    std::optional<std::string> get(const std::string& k) const {
        auto it = values.find(k);
        if (it == values.end()) return std::nullopt;
        return it->second;
    }
};

Args parse_args(int argc, char** argv) {
    static const std::vector<std::string> kWithValue = {
        "input", "mode", "algorithm", "out-dir", "frame-step", "max-frames", "scale", "fps",
        "ref-lat", "ref-lon", "export-scale", "preset", "threads", "altitude", "calibration",
        "altitude-file", "scale-correction", "motion-model", "fixed-step", "target-shift",
        "rotation-threshold", "rotation-cut", "lk", "ins-output-dir", "ins-fps"};
    static const std::vector<std::string> kFlags = {
        "list", "show-video", "no-show-video", "live-plot", "profile", "quiet", "no-figures",
        "rotation-crop", "no-rotation-crop", "no-shadow", "export-ins", "help"};
    static const std::map<char, std::string> kShort = {
        {'i', "input"}, {'m', "mode"}, {'a', "algorithm"}, {'h', "help"}};

    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        std::string key, inline_value;
        bool has_inline = false;
        if (a.rfind("--", 0) == 0) {
            key = a.substr(2);
            const auto eq = key.find('=');
            if (eq != std::string::npos) {
                inline_value = key.substr(eq + 1);
                key = key.substr(0, eq);
                has_inline = true;
            }
        } else if (a.size() == 2 && a[0] == '-' && kShort.count(a[1])) {
            key = kShort.at(a[1]);
        } else {
            throw std::runtime_error("Неизвестный аргумент: " + a);
        }
        if (std::find(kFlags.begin(), kFlags.end(), key) != kFlags.end()) {
            args.flags.push_back(key);
        } else if (std::find(kWithValue.begin(), kWithValue.end(), key) != kWithValue.end()) {
            if (has_inline) {
                args.values[key] = inline_value;
            } else {
                if (i + 1 >= argc) throw std::runtime_error("У ключа --" + key + " нет значения");
                args.values[key] = argv[++i];
            }
        } else {
            throw std::runtime_error("Неизвестный ключ: --" + key);
        }
    }
    return args;
}

double to_double(const std::string& s, const std::string& key) {
    try {
        return std::stod(s);
    } catch (...) {
        throw std::runtime_error("Ключ --" + key + " ожидает число, получено \"" + s + "\"");
    }
}

}  // namespace

int main(int argc, char** argv) {
    vo::init_console();
    try {
        const Args args = parse_args(argc, argv);
        if (args.flag("help")) {
            print_help();
            return 0;
        }

        vo::Config cfg = vo::build_config(std::filesystem::current_path().u8string());
        if (args.flag("list")) {
            vo::check_environment(&cfg);
            return 0;
        }

        if (auto v = args.get("input")) cfg.data_folder = *v;
        if (auto v = args.get("out-dir")) cfg.results_folder = *v;
        if (auto v = args.get("algorithm")) cfg.vo.algorithms = vo::parse_algorithms(*v);
        if (auto v = args.get("frame-step")) cfg.frame_step = static_cast<int>(to_double(*v, "frame-step"));
        if (auto v = args.get("max-frames")) cfg.max_frames = static_cast<long long>(to_double(*v, "max-frames"));
        if (auto v = args.get("scale")) cfg.vo.translation_scale = to_double(*v, "scale");
        if (auto v = args.get("fps")) cfg.export_cfg.frame_rate = to_double(*v, "fps");
        if (auto v = args.get("ref-lat")) cfg.export_cfg.ref_lat = to_double(*v, "ref-lat");
        if (auto v = args.get("ref-lon")) cfg.export_cfg.ref_lon = to_double(*v, "ref-lon");
        if (auto v = args.get("export-scale")) cfg.export_cfg.scale = to_double(*v, "export-scale");
        if (auto v = args.get("preset")) vo::apply_speed_preset(cfg, *v);
        if (args.flag("show-video")) cfg.vis.show_video = true;
        if (args.flag("no-show-video")) cfg.vis.show_video = false;
        if (args.flag("live-plot")) cfg.vis.live_plot = true;
        if (args.flag("profile")) cfg.perf.profile = true;
        if (args.flag("quiet")) cfg.perf.verbose_per_frame = false;
        if (args.flag("no-figures")) cfg.perf.save_match_figures = false;
        if (auto v = args.get("threads")) cfg.perf.num_threads = static_cast<int>(to_double(*v, "threads"));
        if (auto v = args.get("altitude")) {
            cfg.altitude.value = to_double(*v, "altitude");
            cfg.altitude.source = "const";
        }
        if (auto v = args.get("calibration")) cfg.camera.calibration_file = *v;
        if (auto v = args.get("altitude-file")) {
            cfg.altitude.file = *v;
            cfg.altitude.source = "file";
        }
        if (auto v = args.get("scale-correction")) cfg.geometry.scale_correction = to_double(*v, "scale-correction");
        if (auto v = args.get("motion-model")) {
            // Игнорируем; поддерживается только homography
            const std::string m = vo::lower(*v);
            if (m != "homography")
                std::cerr << "[предупреждение] поддерживается только homography, попытка использовать " << m << " проигнорирована\n";
        }
        if (auto v = args.get("lk")) {
            const std::string m = vo::lower(*v);
            if (m != "rotation" && m != "always" && m != "only" && m != "off")
                throw std::runtime_error("--lk: допустимы rotation, always, only, off");
            cfg.lk.mode = m;
        }
        if (auto v = args.get("fixed-step")) {
            cfg.frame_select.mode = "fixed";
            cfg.frame_step = static_cast<int>(to_double(*v, "fixed-step"));
        }
        if (auto v = args.get("target-shift")) cfg.frame_select.target_shift_pct = to_double(*v, "target-shift");
        if (args.flag("rotation-crop")) cfg.rotation_crop.enable = true;
        if (args.flag("no-rotation-crop")) cfg.rotation_crop.enable = false;
        if (args.flag("no-shadow")) cfg.shadow.enable = false;
        // В Python-версии этот ключ писал в неиспользуемое поле ThresholdDeg;
        // здесь он действительно задаёт порог (град/с).
        if (auto v = args.get("rotation-threshold")) cfg.rotation_crop.threshold_dps = to_double(*v, "rotation-threshold");
        if (auto v = args.get("rotation-cut")) {
            const double c = to_double(*v, "rotation-cut");
            cfg.rotation_crop.rotation_cut = {c, c, c, c};
        }
        
        // INS export
        if (args.flag("export-ins")) cfg.ins_export.enable = true;
        if (auto v = args.get("ins-output-dir")) cfg.ins_export.output_dir = *v;
        if (auto v = args.get("ins-fps")) {
            cfg.ins_export.frame_rate = to_double(*v, "ins-fps");
        } else if (auto v = args.get("fps")) {
            // Если --ins-fps не задан, используем --fps
            cfg.ins_export.frame_rate = to_double(*v, "fps");
        }

        if (cfg.perf.num_threads > 0) cv::setNumThreads(cfg.perf.num_threads);
        cv::setUseOptimized(true);
        vo::check_environment(&cfg, false);

        const std::string mode = args.get("mode").value_or("vo");
        if (mode != "vo" && mode != "trajectories")
            throw std::runtime_error("--mode: допустимы vo, trajectories");

        try {
            if (mode == "vo") {
                const vo::VoRun run = vo::run_visual_odometry(cfg);
                int ok = 0;
                for (const auto& r : run.rows)
                    if (vo::starts_with(r.status, "ok")) ++ok;
                double path = 0.0;
                for (size_t i = 1; i < run.positions.size(); ++i)
                    path += std::hypot(run.positions[i][0] - run.positions[i - 1][0],
                                       run.positions[i][1] - run.positions[i - 1][1]);
                std::cout << "\nУспешных шагов: " << ok << "/" << run.rows.size() - 1 << ", длина пути "
                          << vo::fmt("%.1f", path) << " м\n";
            } else {
                std::cout << "\n=== МОНОКУЛЯРНАЯ ВИЗУАЛЬНАЯ ОДОМЕТРИЯ (все методы) ===\n";
                vo::run_all_visual_odometry(cfg);
            }
        } catch (const std::runtime_error& e) {
            std::cout << "[ОШИБКА] " << e.what() << "\n";
            const std::string what = e.what();
            for (const char* w : {"источник", "Папка", "Видеофайл", "кадр", "изображени"})
                if (what.find(w) != std::string::npos) {
                    std::cout << "        Укажите источник кадров ключом --input: папка с картинками, "
                                 "маска \"dir/*.png\", видеофайл или ссылка.\n";
                    break;
                }
            if (what.find("методы") != std::string::npos)
                std::cout << "        Полный список: vo_final --list\n";
            return 1;
        }

        std::cout << "\nГотово. Результаты сохранены в:\n" << cfg.results_folder << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ОШИБКА] " << e.what() << "\n";
        return 2;
    }
}
