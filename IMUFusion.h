#ifndef IMUFUSION_H
#define IMUFUSION_H

#include <array>

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
        void setMeasurementNoise(float rAccel, float rMag);

    private:
        FilterType filterType_;
        float beta_;             // Madgwick algorithm gain
        std::array<float, 4> q_; // Quaternion
        float lastTimestamp_;
        bool hasDirectEuler_;
        float roll_, pitch_, yaw_;

        // Kalman filter variables
        // ADD: Matriz de covariância do estado (6x6)
        float P_[6][6];
        // ADD: Matriz de ruído de processo (6x6)
        float Q_[6][6];
        // ADD: Matriz de ruído de medição do acelerómetro (2x2) - pois só corrigimos roll e pitch
        float RAccel_[2][2];
        // ADD: Vetor de estado (6x1) [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
        float state_[6];

        // Filter update methods
        void updateMadgwick(const IMUData &data, float dt, bool hasMagnetometer);
        void updateKalman(const IMUData &data, float dt, bool hasMagnetometer);
        void updateComplementaryFilter(const IMUData &data, float dt);

        // Helper functions
        float invSqrt(float x) const; // Fast inverse square root
        void normalize(std::array<float, 3> &v) const;

        void matMul66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matAdd66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matTrans66(const float A[6][6], float At[6][6]);
        void matMulVec6(const float A[6][6], const float v[6], float out[6]);
        bool invert2x2(const float M[2][2], float Minv[2][2]);
    };

}

#endif // IMUFUSION_H
