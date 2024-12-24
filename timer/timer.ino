#include "IMUFusion.h"
#include "math.h"

#define G_TO_MPS2  (9.80665f / 1000.0f) // mG to m/s²
#define DEG_TO_RAD (M_PI / 180.0f)     // Degrees/sec to radians/sec

// Cria a instância IMUFusion fora do loop()
IMUFusion::IMUFusion fusion(IMUFusion::FilterType::KALMAN);

float meanTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("A começar! ");

  // Configura o filtro Kalman apenas uma vez
  fusion.setProcessNoise(0.001f);
  fusion.setMeasurementNoise(0.1f, 0.1f);

  fusion.setBeta(0.1f);
}

void loop() {
  static int counter = 0; // Usa uma variável estática para contar as iterações

  if (counter < 1000) {
    float accel_x = 100 * G_TO_MPS2;
    float accel_y = 0 * G_TO_MPS2;
    float accel_z = 0 * G_TO_MPS2;

    // Convert gyroscope data to radians/sec
    float gyro_x = 0 * DEG_TO_RAD;
    float gyro_y = 0.552 * DEG_TO_RAD;
    float gyro_z = 0.5325 * DEG_TO_RAD;
    float timestamp = 6287908.0 + counter;

    IMUFusion::IMUData imuData;

    imuData.accelerometer[0] = accel_x;
    imuData.accelerometer[1] = accel_y;
    imuData.accelerometer[2] = accel_z;
    imuData.gyroscope[0] = gyro_x;
    imuData.gyroscope[1] = gyro_y;
    imuData.gyroscope[2] = gyro_z;

    float timeBefore = millis();

    fusion.update(imuData, timestamp);

    float timeElapsed = millis() - timeBefore;
    meanTime = meanTime + timeElapsed;
    Serial.print("Iteracao n: ");
    Serial.print(counter);
    Serial.print(" | tempo: ");
    Serial.print(timeElapsed);
    Serial.print(" ms");
    Serial.println();

    
    counter++;
  } else {
    Serial.println("Fim da execucao.");

    meanTime = meanTime / 1000;
    Serial.println("Média de tempo: ");
    Serial.print(meanTime);
    Serial.print(" ms");
    while (1); // Para o loop ao atingir 1000 iterações
  }
}
