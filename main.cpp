#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath> // For M_PI
#include <chrono>
#include "IMUFusion.h"

int main()
{
    // Initialize with Kalman filter
    IMUFusion::IMUFusion fusion(IMUFusion::FilterType::KALMAN);

    // Set Kalman filter parameters
    fusion.setProcessNoise(0.01f);
    fusion.setMeasurementNoise(0.1f, 0.2f);

    // Open the data file
    std::ifstream file("dados/teste_estatico_com_video_2min_de_movimento.txt");
    if (!file.is_open())
    {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    std::string line;
    float timestamp;
    float accel_x_mG, accel_y_mG, accel_z_mG;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float mag_x, mag_y, mag_z;

    // Constants for unit conversion
    const float G_TO_MPS2 = 9.80665f / 1000.0f; // mG to m/s²
    const float DEG_TO_RAD = M_PI / 180.0f;     // Degrees/sec to radians/sec

    // Create IMUData structure
    IMUFusion::IMUData imuData;

    fusion.setWindowSize(7);

    // Read the file line by line
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        // Parse data from the line
        if (!(iss >> timestamp >> accel_x_mG >> accel_y_mG >> accel_z_mG >> gyro_x_dps >> gyro_y_dps >> gyro_z_dps >> mag_x >> mag_y >> mag_z))
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

        imuData.accelerometer[0] = accel_x;
        imuData.accelerometer[1] = accel_y;
        imuData.accelerometer[2] = accel_z;
        imuData.gyroscope[0] = gyro_x;
        imuData.gyroscope[1] = gyro_y;
        imuData.gyroscope[2] = gyro_z;
        // Magnemeter has a different coordinate system, map it to the same as the accelerometer and gyroscope
        imuData.magnetometer[0] = mag_y;
        imuData.magnetometer[1] = mag_x;
        imuData.magnetometer[2] = -mag_z;

        // Update the sensor fusion filter

        /*
        float currentTime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
        */
        // fusion.lowPassFilter(imuData, 0.5f, 0.5f, 0.5f);
        fusion.movingAverageFilter(imuData);

        std::ofstream outFile("filtered_data.txt", std::ios::app);
        outFile << timestamp << " "
                << imuData.accelerometer[0] / G_TO_MPS2 << " "
                << imuData.accelerometer[1] / G_TO_MPS2 << " "
                << imuData.accelerometer[2] / G_TO_MPS2 << " "
                << imuData.gyroscope[0] / DEG_TO_RAD << " "
                << imuData.gyroscope[1] / DEG_TO_RAD << " "
                << imuData.gyroscope[2] / DEG_TO_RAD << " "
                << imuData.magnetometer[0] << " "
                << imuData.magnetometer[1] << " "
                << imuData.magnetometer[2] << "\n";

        fusion.update(imuData, timestamp, true);

        /*
        float timespent = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count() -
                          currentTime;

        std::cout << "Time spent: " << timespent << "us" << std::endl;*/
        // Retrieve the current orientation as Euler angles
        float eulerAngles[3];
        fusion.getEulerAngles(eulerAngles);

        // Open the output file in append
        /*
       std::ofstream outFile("orientation_data.txt", std::ios::app);
       if (!outFile.is_open())
       {
           std::cerr << "Error opening output file." << std::endl;
           return 1;
       }

       // Write the timestamp and Euler angles to the file

       outFile << timestamp << " "
               << eulerAngles[0] * 180.0 / M_PI << " "
               << eulerAngles[1] * 180.0 / M_PI << " "
               << eulerAngles[2] * 180.0 / M_PI << "\n";

       // Close the output file
       outFile.close();*/

        // Print the orientation (roll, pitch, yaw) in degrees
        std::cout << "Timestamp: " << timestamp << " us | "
                  << "Roll: " << eulerAngles[0] * 180.0 / M_PI << " deg, "
                  << "Pitch: " << eulerAngles[1] * 180.0 / M_PI << " deg, "
                  << "Yaw: " << eulerAngles[2] * 180.0 / M_PI << " deg" << std::endl;
    }

    file.close();
    return 0;
}
