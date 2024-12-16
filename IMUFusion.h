#ifndef IMUFUSION_H
#define IMUFUSION_H

#include <array>
#include <Eigen/Dense>

namespace IMUFusion
{
    enum class FilterType
    {
        MADGWICK,
        KALMAN,
        COMPLEMENTARY
    };

    struct IMUData
    {
        std::array<float, 3> accelerometer; // ax, ay, az
        std::array<float, 3> gyroscope;     // gx, gy, gz (rad/s)
        std::array<float, 3> magnetometer;  // mx, my, mz (optional)
    };

    class IMUFusion
    {
    public:
        // Updated default beta value to match the reference implementation
        IMUFusion(FilterType type = FilterType::MADGWICK, float beta = 0.041f);

        // Update with new IMU data and current timestamp (in milliseconds)
        void update(const IMUData &data, uint64_t timestamp, bool hasMagnetometer = false);

        // Get orientation as a quaternion
        std::array<float, 4> getQuaternion() const;

        // Get orientation as Euler angles (roll, pitch, yaw in radians)
        std::array<float, 3> getEulerAngles() const;

        void setBeta(float beta);
        void setProcessNoise(float q);
        void setMeasurementNoise(float r, float m);

    private:
        FilterType filterType_;
        float beta_;             // Madgwick algorithm gain
        std::array<float, 4> q_; // Quaternion
        uint64_t lastTimestamp_;
        bool hasDirectEuler_;
        float roll_, pitch_, yaw_;

        // Kalman filter variables
        Eigen::MatrixXf P_;      // State covariance
        Eigen::MatrixXf Q_;      // Process noise
        Eigen::MatrixXf RAccel_; // Measurement noise
        Eigen::MatrixXf RMag_;   // Measurement noise

        // Filter update methods
        void updateMadgwick(const IMUData &data, float dt, bool hasMagnetometer);
        void updateKalman(const IMUData &data, float dt, bool hasMagnetometer);
        void updateComplementaryFilter(const IMUData &data, float dt);

        // Helper functions
        float invSqrt(float x) const; // Fast inverse square root
        void normalize(std::array<float, 3> &v) const;
        void normalizeQuaternion();
    };

}

#endif // IMUFUSION_H
