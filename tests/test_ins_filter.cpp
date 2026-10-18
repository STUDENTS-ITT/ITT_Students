// test_ins_filter.cpp — Инварианты фильтра Калмана 15-го порядка.
//
// Проверяются свойства, которые «тихо» ломаются при неверной размерности
// матриц: симметрия и положительность P, обнуление x после коррекции,
// уменьшение следа ковариации, диал��ги Q и R.

#include <cmath>

#include <QtTest>

#include "core/ins/ins_filter.h"
#include "core/math_lib/matrix_ops.h"
#include "core/navigation/aligner.h"
#include "core/utils/constants.h"

namespace
{

// Максимальное расхождение P и Pᵀ.
double asymmetry(const Matrix &P, int n)
{
    double worst = 0.0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            worst = qMax(worst, std::fabs(at(P, i, j, n) - at(P, j, i, n)));
        }
    }
    return worst;
}

double traceOf(const Matrix &P, int n)
{
    double tr = 0.0;
    for (int i = 0; i < n; i++) tr += at(P, i, i, n);
    return tr;
}

bool diagonalPositive(const Matrix &P, int n)
{
    for (int i = 0; i < n; i++)
    {
        if (!(at(P, i, i, n) >= 0.0)) return false;
    }
    return true;
}

// Начальная ковариация из выставки (диагональная).
Matrix makeP0()
{
    return nav::initialCovariance(22.4 * DEG_TO_RAD);
}

} // namespace

class TestInsFilter : public QObject
{
    Q_OBJECT

private slots:
    void processNoiseDiagonal();
    void measurementNoiseDiagonal();
    void predictKeepsCovarianceValid();
    void predictPropagatesState();
    void correctWithZeroInnovation();
    void correctReducesUncertainty();
    void correctAlignsState();
    void correctAccelIsolatedFromPosition();
    void correctAccelSeparatesBiasFromTilt();
    void resetCovarianceSymmetrizes();
    void initialCovarianceIsSymmetricWithTiltBiasCoupling();
};

void TestInsFilter::processNoiseDiagonal()
{
    const double T = 0.005;
    const auto cfg = ins::kalman_cfg;
    const Matrix Q = ins::Qj_matrix(T);

    QCOMPARE(Q.size(), std::size_t(ins::KF_STATE * ins::KF_STATE));

    // Координаты шума процесса не имеют.
    for (int i = 0; i < 3; i++) QCOMPARE(at(Q, i, i, ins::KF_STATE), 0.0);
    for (int i = 3; i < 6; i++)
        QCOMPARE(at(Q, i, i, ins::KF_STATE), cfg.sig_a * cfg.sig_a * T);
    for (int i = 6; i < 9; i++)
        QCOMPARE(at(Q, i, i, ins::KF_STATE), cfg.sig_g * cfg.sig_g * T);
    for (int i = 9; i < 12; i++)
        QCOMPARE(at(Q, i, i, ins::KF_STATE), cfg.sig_ba * cfg.sig_ba * T);
    for (int i = 12; i < 15; i++)
        QCOMPARE(at(Q, i, i, ins::KF_STATE), cfg.sig_bg * cfg.sig_bg * T);

    // Вне диагонали — нули.
    for (int i = 0; i < ins::KF_STATE; i++)
    {
        for (int j = 0; j < ins::KF_STATE; j++)
        {
            if (i != j) QCOMPARE(at(Q, i, j, ins::KF_STATE), 0.0);
        }
    }
}

void TestInsFilter::measurementNoiseDiagonal()
{
    const double sp = 0.3, sr = 0.5;
    const Matrix R = ins::Rj_matrix(sp, sr, sr);
    QCOMPARE(R.size(), std::size_t(ins::KF_MEAS * ins::KF_MEAS));
    QCOMPARE(R.size(), std::size_t(81));

    const auto cfg = ins::kalman_cfg;
    QCOMPARE(at(R, 0, 0, ins::KF_MEAS), cfg.sig_pos * cfg.sig_pos);
    QCOMPARE(at(R, 1, 1, ins::KF_MEAS), cfg.sig_pos * cfg.sig_pos);
    QCOMPARE(at(R, 2, 2, ins::KF_MEAS), cfg.sig_h * cfg.sig_h);
    for (int i = 3; i < 6; i++)
        QCOMPARE(at(R, i, i, ins::KF_MEAS), cfg.sig_v * cfg.sig_v);
    QCOMPARE(at(R, 6, 6, ins::KF_MEAS), sp * sp);
    QCOMPARE(at(R, 7, 7, ins::KF_MEAS), sr * sr);
    QCOMPARE(at(R, 8, 8, ins::KF_MEAS), sr * sr);
}

void TestInsFilter::predictKeepsCovarianceValid()
{
    Matrix P = makeP0();
    Vector x(ins::KF_STATE, 0.0);
    const Matrix C = bodyToNavMatrix(0.5, 0.1, -0.05);
    const Vector f_nav = {0.0, 9.8, 0.0};
    ins::Attitude att{0.5, 0.1, -0.05};

    for (int i = 0; i < 100; i++)
    {
        ins::predict(0.005, 22.4 * DEG_TO_RAD, 100.0, C, f_nav, att, x, P);
    }

    QVERIFY(diagonalPositive(P, ins::KF_STATE));
    QVERIFY(traceOf(P, ins::KF_STATE) > 0.0);
    QCOMPARE(x.size(), std::size_t(ins::KF_STATE));
}

void TestInsFilter::predictPropagatesState()
{
    Vector x(ins::KF_STATE, 0.0);
    x[3] = 1.0; // ошибка скорости на север
    Matrix P = makeP0();

    const Matrix C = bodyToNavMatrix(0.0, 0.0, 0.0);
    const Vector f_nav = {0.0, 9.8, 0.0};
    ins::Attitude att{0.0, 0.0, 0.0};

    const double dt = 1.0;
    ins::predict(dt, 0.0, 0.0, C, f_nav, att, x, P);

    // x = F·x: ошибка скорости должна перейти в ошибку широты (dt/R).
    QVERIFY(qAbs(x[0] - dt / R_EARTH) < 1e-15);
    // Остальные компоненты не изменились (нет связей при нулевых углах).
    QCOMPARE(x[1], 0.0);
    QCOMPARE(x[2], 0.0);
}

void TestInsFilter::correctWithZeroInnovation()
{
    // Нулевая инновация (БИНС == СНС) не должна двигать оценку.
    Vector x(ins::KF_STATE, 0.0);
    Matrix P = makeP0();
    const Matrix P0 = P;

    Vector bins(ins::KF_MEAS, 0.0);
    Vector sns(ins::KF_MEAS, 0.0);

    ins::correct(bins, sns, x, P, 0.01, 1e-3, 1e-3);

    for (int i = 0; i < ins::KF_STATE; i++)
    {
        QVERIFY(qAbs(x[i]) < 1e-15);
    }
    QVERIFY(diagonalPositive(P, ins::KF_STATE));
    QVERIFY(traceOf(P, ins::KF_STATE) <= traceOf(P0, ins::KF_STATE) + 1e-12);
}

void TestInsFilter::correctReducesUncertainty()
{
    Matrix P = makeP0();
    const double before = traceOf(P, ins::KF_STATE);

    Vector x(ins::KF_STATE, 0.0);
    Vector bins(ins::KF_MEAS, 0.0);
    Vector sns(ins::KF_MEAS, 0.0);

    ins::correct(bins, sns, x, P, 0.01, 1e-3, 1e-3);

    QVERIFY(traceOf(P, ins::KF_STATE) < before);
    QVERIFY(diagonalPositive(P, ins::KF_STATE));
}

void TestInsFilter::correctAlignsState()
{
    // Большая инновация по широте должна уменьшаться после коррекции.
    Vector x(ins::KF_STATE, 0.0);
    Matrix P = makeP0();

    Vector bins(ins::KF_MEAS, 0.0);
    Vector sns(ins::KF_MEAS, 0.0);
    bins[0] = 1e-6; // БИНС ошибся на широте на 1e-6 рад (~0.6 м)
    sns[0] = 0.0;

    ins::correct(bins, sns, x, P, 0.01, 1e-3, 1e-3);

    QVERIFY(x[0] > 0.0);
    // После коррекции ошибка широты меньше исходной инновации.
    QVERIFY(std::fabs(x[0]) < 1e-6);
    // Симметрия ковариации в Joseph-форме.
    QVERIFY(asymmetry(P, ins::KF_STATE) < 1e-18);
}

void TestInsFilter::correctAccelIsolatedFromPosition()
{
    // correctAccel не должен трогать pos/vel/hdg (П2): строки K[0..6] обнулены.
    Vector x(ins::KF_STATE, 0.0);
    Matrix P = makeP0();
    const Matrix P0 = P;

    // Реалистичная корреляция «ошибка скорости ↔ ошибка тангажа»: именно она
    // «протекла» бы в x[3..5] через K = P·Hᵀ, если бы строки K[0..6] не
    // обнулялись.  Пары подобраны так, чтобы P осталась положительно
    // определённой (σ_vel = 2.5 м/с против σ_θ = 1°).
    for (int i = 3; i < 6; i++)
    {
        at(P, i, 7, ins::KF_STATE) = 0.01;
        at(P, 7, i, ins::KF_STATE) = 0.01;
        at(P, i, 8, ins::KF_STATE) = 0.01;
        at(P, 8, i, ins::KF_STATE) = 0.01;
    }

    // Аппарат стоит ровно: C = I, тяжесть видна акселю по оси Y тела.
    ins::Attitude att;
    att.heading = 0.0;
    att.pitch = 0.0;
    att.roll = 0.0;
    const Matrix C = bodyToNavMatrix(att.heading, att.pitch, att.roll);
    const Vector f_nav = {0.0, 9.81, 0.0};

    // Инновация отвечает ошибке тангажа 0.02 рад: при C = I h − y = g·δθ·e_x.
    const Vector z = {9.81 * 0.02, 0.0, 0.0};
    ins::correctAccel(C, f_nav, att, z, 1e-3, x, P);

    for (int i = 0; i < 7; i++)
    {
        QVERIFY(qAbs(x[i]) < 1e-18);
    }
    // Невязка объяснена целиком: g·x[7] − x[9] = z.  Как она делится между
    // углом и смещением — решает P₀, и при честной корреляции R3 аксель почти
    // не двигает ba (направление измерения почти ортогонально разности
    // «наклон − ba/g»); это разобрано в correctAccelSeparatesBiasFromTilt.
    QVERIFY(x[7] > 0.01);
    QVERIFY(qAbs(9.81 * x[7] - x[9] - 9.81 * 0.02) < 1e-2);
    QVERIFY(diagonalPositive(P, ins::KF_STATE));

    // Ковариация координат не должна уменьшиться из-за аксель-коррекции.
    for (int i = 0; i < 3; i++)
    {
        QVERIFY(at(P, i, i, ins::KF_STATE) >= at(P0, i, i, ins::KF_STATE) - 1e-15);
    }
}

void TestInsFilter::correctAccelSeparatesBiasFromTilt()
{
    // Ошибка тангажа и смещение акселя дают инновацию вдоль одной оси тела,
    // но столбцы H у них разные (∂h/∂θ = g, ∂h/∂ba = −1).  При заведомо
    // точной выставке (малая σ угла) всё наблюдение должно уйти в смещение —
    // это и есть R2: аксель-апдейт обязан уметь отдавать инновацию в ba.
    Vector x(ins::KF_STATE, 0.0);
    Matrix P = makeP0();
    // Задаём блок (θ, ba_X) заново: при заведомо точной выставке
    // (σ(θ) = 0.001 рад) вся невязка должна уйти в смещение σ(ba_X) = 0.05 м/с².
    // Корреляцию R3 здесь обнуляем — иначе 2×2 получился бы не положительно
    // определённым (P₀ рассчитана на согласованные σ).
    at(P, 7, 7, ins::KF_STATE) = 1e-6;
    at(P, 7, 9, ins::KF_STATE) = 0.0;
    at(P, 9, 7, ins::KF_STATE) = 0.0;
    at(P, 9, 9, ins::KF_STATE) = 0.05 * 0.05;

    ins::Attitude att;
    att.heading = 0.0;
    att.pitch = 0.0;
    att.roll = 0.0;
    const Matrix C = bodyToNavMatrix(att.heading, att.pitch, att.roll);
    const Vector f_nav = {0.0, 9.81, 0.0};

    // Истина: b̂a = 0, аппарат ровный.  Оценка завысила b̂a_X на 0.05 м/с²,
    // значит h − y = +0.05 по оси X тела.
    const Vector z = {0.05, 0.0, 0.0};
    ins::correctAccel(C, f_nav, att, z, 1e-3, x, P);

    // Инъекция смещения — сложение x[9], ошибка была +0.05, значит x[9] ≈ −0.05.
    QVERIFY(x[9] < -0.04);
    QVERIFY(qAbs(x[7]) < 0.005);
}

void TestInsFilter::resetCovarianceSymmetrizes()
{
    Matrix P = makeP0();
    // Вносим асимметрию и отрицательную ди��гональ.
    at(P, 1, 2, ins::KF_STATE) = 5.0;
    at(P, 2, 1, ins::KF_STATE) = -5.0;
    at(P, 4, 4, ins::KF_STATE) = -1.0;

    ins::apply_reset_covariance(P);

    QVERIFY(asymmetry(P, ins::KF_STATE) < 1e-18);
    QVERIFY(diagonalPositive(P, ins::KF_STATE));
    QCOMPARE(at(P, 4, 4, ins::KF_STATE), 1e-15);
}

void TestInsFilter::initialCovarianceIsSymmetricWithTiltBiasCoupling()
{
    const double lat = 22.4 * DEG_TO_RAD;
    const Matrix P = nav::initialCovariance(lat);
    QCOMPARE(P.size(), std::size_t(ins::KF_STATE * ins::KF_STATE));

    for (int i = 0; i < ins::KF_STATE; i++)
    {
        QVERIFY(at(P, i, i, ins::KF_STATE) > 0.0);
        for (int j = 0; j < ins::KF_STATE; j++)
        {
            if (i == j) continue;
            // R3: единственные ненулевые внедиагональные элементы — связь
            // «наклон ↔ горизонтальное смещение акселя», внесённая выставкой.
            const bool coupled = (i == 7 && j == 9) || (i == 9 && j == 7) ||
                                 (i == 8 && j == 11) || (i == 11 && j == 8);
            if (!coupled) QCOMPARE(at(P, i, j, ins::KF_STATE), 0.0);
        }
    }

    QVERIFY(asymmetry(P, ins::KF_STATE) < 1e-18);

    // Знаки связи: тангаж завышен вместе с ba_X, крен — в противофазе с ba_Z.
    QVERIFY(at(P, 7, 9, ins::KF_STATE) > 0.0);
    QVERIFY(at(P, 8, 11, ins::KF_STATE) < 0.0);

    // Величина связи — σ_ba²/g (g выведен в самой матрице).
    const double g = normalGravity(lat, 0.0);
    const double expected = at(P, 9, 9, ins::KF_STATE) / g;
    QVERIFY(std::fabs(at(P, 7, 9, ins::KF_STATE) - expected) < expected * 1e-9);
    QVERIFY(std::fabs(at(P, 8, 11, ins::KF_STATE) + expected) < expected * 1e-9);

    // Связь не выводит блок «тангаж/ba_X» из положительной определённости.
    const double det_pt = at(P, 7, 7, ins::KF_STATE) * at(P, 9, 9, ins::KF_STATE) -
                          at(P, 7, 9, ins::KF_STATE) * at(P, 9, 7, ins::KF_STATE);
    QVERIFY(det_pt > 0.0);
}

QTEST_MAIN(TestInsFilter)
#include "test_ins_filter.moc"