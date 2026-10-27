// kalman_filter.hpp — 15-state Extended Kalman Filter for VO+INS fusion
// Adapted from ITT_Students add-kalman-report branch
//
// State vector (15 components):
//   x[0..2]   — position errors (δφ, δλ, δh) [rad, rad, m]
//   x[3..5]   — velocity errors (δVn, δVh, δVe) [m/s]
//   x[6..8]   — attitude errors (δψ, δθ, δγ) [rad]
//   x[9..11]  — accelerometer bias (δba) [m/s²]
//   x[12..14] — gyroscope bias (δbg) [rad/s]
//
// Measurement vector (9 components):
//   z[0..2] — position difference (INS − VO)
//   z[3..5] — velocity difference (INS − VO)
//   z[6..8] — attitude difference (INS − VO)

#pragma once

#include <cmath>
#include <vector>

namespace vo {
namespace fusion {

constexpr int KF_STATE = 15;
constexpr int KF_MEAS = 9;

// Kalman filter configuration (noise parameters)
struct KalmanConfig {
    double sig_g = 0.002;           // gyroscope noise, rad/√s
    double sig_a = 0.15;            // accelerometer noise, m/s²/√s
    double sig_bg = 1.05526e-05;    // gyroscope bias drift, rad/s/√s
    double sig_ba = 1.10162e-05;    // accelerometer bias drift, m/s²/√s
    double sig_pos_m = 50.0;        // VO position noise, m
    double sig_h = 2.0;             // VO altitude noise, m
    double sig_v = 1.0;             // VO velocity noise, m/s
    double sig_hdg = 0.2;           // VO heading noise, rad
    
    // Adaptive trust on turns/water
    double turn_w0 = 0.25;          // turn threshold, rad/s
    double turn_sigma_gain = 8.0;   // sigma growth per rad/s above threshold
    double turn_sigma_max = 5.0;    // max sigma multiplier on turns
    
    bool over_water = false;        // reduce VO trust over water
    double water_sigma_mult = 3.0;  // sigma multiplier over water
};

// 3D vector helper
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

// Attitude (Euler angles)
struct Attitude {
    double heading = 0;  // yaw (ψ), rad
    double pitch = 0;    // θ, rad
    double roll = 0;     // γ, rad
};

// Navigation state
struct NavState {
    double lat = 0;      // latitude, rad
    double lon = 0;      // longitude, rad
    double alt = 0;      // altitude, m
    Vec3 velocity;       // [Vn, Vh, Ve], m/s
    Attitude attitude;
    
    Vec3 ba;             // accelerometer bias, m/s²
    Vec3 bg;             // gyroscope bias, rad/s
    
    std::vector<double> x;  // 15-state error vector
    std::vector<double> P;  // 15×15 covariance matrix (row-major)
    
    double time_prev = 0;
    Vec3 V_dot_prev;
    double lat_dot_prev = 0, lon_dot_prev = 0, alt_dot_prev = 0;
};

// VO measurement (from visual odometry)
struct VoMeasurement {
    double lat = 0, lon = 0, alt = 0;     // position, rad/rad/m
    Vec3 velocity;                         // [Vn, Vh, Ve], m/s
    double heading = 0;                    // rad
    double pitch = 0, roll = 0;            // rad (usually from accelerometer)
    bool valid = false;
    
    // Quality indicators
    int num_matches = 0;
    int num_inliers = 0;
    double shift_ratio = 0.0;  // frame shift as fraction of frame width
};

// IMU measurement
struct ImuMeasurement {
    double time = 0;              // seconds
    Vec3 gyro;                    // angular velocity, rad/s
    Vec3 accel;                   // linear acceleration, m/s²
};

// Kalman filter for VO+INS fusion
class KalmanFusion {
public:
    explicit KalmanFusion(const KalmanConfig& cfg = KalmanConfig());
    
    // Initialize state from first VO measurement and IMU alignment
    void initialize(const VoMeasurement& vo0, double lat_rad, double lon_rad, double alt_m,
                    const Attitude& att0, const Vec3& ba0, const Vec3& bg0);
    
    // Prediction step (INS propagation, called at IMU rate ~200 Hz)
    void predict(const ImuMeasurement& imu);
    
    // Correction step (VO update, called at VO rate ~10 Hz)
    void correct(const VoMeasurement& vo, double yaw_rate_rad_s);
    
    // Get corrected navigation state
    NavState getState() const { return state_; }
    
    // Get error vector for diagnostics
    std::vector<double> getErrorVector() const { return state_.x; }
    
private:
    KalmanConfig cfg_;
    NavState state_;
    bool initialized_ = false;
    
    // Matrix operations
    std::vector<double> Fj_matrix(double T, const std::vector<double>& C,
                                   const Vec3& f_nav);
    std::vector<double> Qj_matrix(double T);
    std::vector<double> Rj_matrix(double sig_pos, double sig_h, double sig_v,
                                   double sig_hdg, double sig_pitch, double sig_roll);
    
    void applyCorrections();
    void symmetrizeCovariance();
    
    // Coordinate transformations
    std::vector<double> bodyToNavMatrix(double heading, double pitch, double roll);
    Vec3 bodyToNav(const std::vector<double>& C, const Vec3& v_body);
    Vec3 navToBody(const std::vector<double>& C, const Vec3& v_nav);
};

} // namespace fusion
} // namespace vo
