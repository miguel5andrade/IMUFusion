#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath> // For M_PI
#include <chrono>
#include "IMUFusion.h"

int main()
{
    // Initialize with Kalman filter
    IMUFusion::IMUFusion fusion(IMUFusion::FilterType::COMPLEMENTARY);

    // Set Kalman filter parameters
    fusion.setProcessNoise(0.001f);
    fusion.setMeasurementNoise(0.1f, 0.1f);

    fusion.setBeta(0.1f); // Set the algorithm gain
    // Open the data file
    std::ifstream file("LAB1_3.txt");
    if (!file.is_open())
    {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    std::string line;
    float timestamp;
    float accel_x_mG, accel_y_mG, accel_z_mG;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    // Constants for unit conversion
    const float G_TO_MPS2 = 9.80665f / 1000.0f; // mG to m/s²
    const float DEG_TO_RAD = M_PI / 180.0f;     // Degrees/sec to radians/sec

    // Read the file line by line
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        // Parse data from the line
        if (!(iss >> timestamp >> accel_x_mG >> accel_y_mG >> accel_z_mG >> gyro_x_dps >> gyro_y_dps >> gyro_z_dps))
        {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }

        // Convert accelerometer data to m/s²
        float accel_x = accel_x_mG * G_TO_MPS2;
        float accel_y = accel_y_mG * G_TO_MPS2;
        float accel_z = accel_z_mG * G_TO_MPS2;

        // Convert gyroscope data to radians/sec
        float gyro_x = gyro_x_dps * DEG_TO_RAD;
        float gyro_y = gyro_y_dps * DEG_TO_RAD;
        float gyro_z = gyro_z_dps * DEG_TO_RAD;

        // Create IMUData structure
        IMUFusion::IMUData imuData;
        imuData.accelerometer = {accel_x, accel_y, accel_z};
        imuData.gyroscope = {gyro_x, gyro_y, gyro_z};

        // Update the sensor fusion filter
        auto currentTime = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();

        fusion.update(imuData, timestamp, false);

        auto timespent = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count() -
                         currentTime;

        // std::cout << "Time spent: " << timespent << "us" << std::endl;
        // Retrieve the current orientation as Euler angles
        auto eulerAngles = fusion.getEulerAngles();

        // Print the orientation (roll, pitch, yaw) in degrees
        std::cout << "Timestamp: " << timestamp << " us | "
                  << "Roll: " << eulerAngles[0] * 180.0 / M_PI << " deg, "
                  << "Pitch: " << eulerAngles[1] * 180.0 / M_PI << " deg, "
                  << "Yaw: " << eulerAngles[2] * 180.0 / M_PI << " deg" << std::endl;
    }

    file.close();
    return 0;
}
