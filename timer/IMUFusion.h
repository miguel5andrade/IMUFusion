#ifndef IMUFUSION_H
#define IMUFUSION_H
#include "math.h"
namespace IMUFusion
{
    /**
     * @brief The type of filters available to use
     */
    enum class FilterType
    {
        EKF,          ///< Extended Kalman filter for a more precise orientation estimation
        COMPLEMENTARY ///< Complementary filter for a lightweight orientation estimation
    };

    /**
     * @brief Structure to hold the IMU data, accelerometer, gyroscope, and magnetometer. Attention: Its assumed that the coordinate system for the accelerometer, gyroscope, and magnetometer are the same. If not, the data must be transformed to the same coordinate system.
     */
    struct IMUData
    {
        float accelerometer[3]; // ax, ay, az (m/s^2)
        float gyroscope[3];     // gx, gy, gz (rad/s)
        float magnetometer[3];  // mx, my, mz (units are not relevant, has to measure the magnetic field)
    };

    /**
     * @brief Class implementing sensor fusion using Kalman and complementary filters.
     */
    class IMUFusion
    {
    public:
        /**
         * @brief Constructor to initialize the filter type.
         * @param type Filter type (Kalman or Complementary).
         */
        IMUFusion(FilterType type = FilterType::KALMAN);

        /**
         * @brief Destructor, frees allocated memory.
         */
        ~IMUFusion();

        /**
         * @brief Update the filter with new IMU data.
         * @param data IMU sensor data.
         * @param timestamp Current timestamp in milliseconds.
         * @param hasMagnetometer Indicates if magnetometer data is available, default is false.
         */
        void update(IMUData &data, float timestamp, bool hasMagnetometer = false);

        /**
         * @brief Apply a low-pass filter to the IMU data, the higher the alpha, the higher the filtering.
         * @param data IMU sensor data.
         * @param accelerometer_alpha Alpha value for accelerometer data.
         * @param gyro_alpha Alpha value for gyroscope data.
         * @param magnetometer_alpha Alpha value for magnetometer data.
         */
        void lowPassFilter(IMUData &data, float accelerometer_alpha = 0.1f, float gyro_alpha = 0.01f, float magnetometer_alpha = 0.1f);

        /**
         * @brief Apply a moving average filter to the IMU data.
         * @param data IMU sensor data.
         */
        void movingAverageFilter(IMUData &data);

        /**
         * @brief Sets the window size for the movingAverageFilter(), allocates the buffers. Can only be called once on a IMUFusion object
         * @param windowSize Size of the window used in the movingAverageFilter.
         */
        void setWindowSize(int windowSize);

        /**
         * @brief Retrieve the current orientation as Euler angles, the convention used is ZYX.
         * @param angles Array to store the Euler angles (roll (around X), pitch (around Y), yaw (around Z)).
         */
        void getEulerAngles(float angles[3]) const;

        /**
         * @brief Set the process noise for the Kalman filter.
         * @param q Process noise value.
         */
        void setProcessNoise(float q = 0.01f);

        /**
         * @brief Set the measurement noise for the Kalman filter.
         * @param rAccel Measurement noise for the accelerometer.
         * @param rMag Measurement noise for the magnetometer.
         */
        void setMeasurementNoise(float rAccel = 0.1f, float rMag = 0.2f);

        /**
         * @brief Set the alpha value for the complementary filter.
         * @param alpha Alpha value for the complementary filter.
         */
        void setAlpha(float alpha = 0.96f);

    private:
        FilterType filterType_;    ///< Type of filter used.
        float alpha_;              ///< Weight factor for complementary filter.
        float lastTimestamp_;      ///< Last update timestamp.
        float roll_, pitch_, yaw_; ///< Orientation angles.

        // Kalman filter variables
        float P_[6][6];      ///< State covariance matrix.
        float Q_[6][6];      ///< Process noise covariance matrix.
        float RAccel_[2][2]; ///< Measurement noise covariance matrix for accelerometer.
        float state_[6];     ///< State vector: roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate.
        float rMag_;         ///< Magnetometer measurement noise.

        // Variable for median filter
        int windowSize_;
        float *accelBuffer[3];
        float *gyroBuffer[3];
        float *magBuffer[3];

        int accelIndex;
        int gyroIndex;
        int magIndex;
        int bufferCount;

        float accelSum[3];
        float gyroSum[3];
        float magSum[3];

        IMUData previousData; ///< Previous data for low-pass filtering.

        // Filter update methods
        void updateKalman(IMUData &data, float dt, bool hasMagnetometer);
        void updateComplementaryFilter(IMUData &data, float dt);

        void matMul66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matAdd66(const float A[6][6], const float B[6][6], float C[6][6]);
        void matTrans66(const float A[6][6], float At[6][6]);
        void matMulVec6(const float A[6][6], const float v[6], float out[6]);
        bool invert2x2(const float M[2][2], float Minv[2][2]);
    };

}

#endif // IMUFUSION_H
