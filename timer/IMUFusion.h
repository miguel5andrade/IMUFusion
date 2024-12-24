#ifndef IMUFUSION_H
#define IMUFUSION_H

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
        float accelerometer[3]; // ax, ay, az
        float gyroscope[3];     // gx, gy, gz (rad/s)
        float magnetometer[3];  // mx, my, mz (optional)
    };

    class IMUFusion
    {
    public:
        IMUFusion(FilterType type = FilterType::COMPLEMENTARY, float beta = 0.041f);

        // Update with new IMU data and current timestamp (in milliseconds)
        void update(IMUData &data, float timestamp, bool hasMagnetometer = false);

        // Get orientation as a quaternion
        void getQuaternion(float q[4]) const;

        // Get orientation as Euler angles (roll, pitch, yaw in radians)
        void getEulerAngles(float angles[3]) const;

        void setBeta(float beta);
        void setProcessNoise(float q);
        void setMeasurementNoise(float rAccel, float rMag);

    private:
        FilterType filterType_;
        float beta_; // Madgwick algorithm gain
        float q_[4]; // Quaternion
        float lastTimestamp_;
        bool hasDirectEuler_;
        float roll_, pitch_, yaw_;

        // Kalman filter variables
        float P_[6][6];
        float Q_[6][6];
        float RAccel_[2][2];
        float state_[6];

        // Filter update methods
        void updateMadgwick(IMUData &data, float dt, bool hasMagnetometer);
        void updateKalman(IMUData &data, float dt, bool hasMagnetometer);
        void updateComplementaryFilter(IMUData &data, float dt);

        // Helper functions
        float invSqrt(float x) const;
        void normalize(float v[3]) const;

        void matMul66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matAdd66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matTrans66(const float A[6][6], float At[6][6]);
        void matMulVec6(const float A[6][6], const float v[6], float out[6]);
        bool invert2x2(const float M[2][2], float Minv[2][2]);
    };

}

#endif // IMUFUSION_H
