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
  fusion.setAlpha(0.95f);
  fusion.setWindowSize(3);

}

void loop() {
  static int counter = 0; // Usa uma variável estática para contar as iterações

  if (counter < 10000) {
    float timestamp = 6287908.0 + counter;

    IMUFusion::IMUData imuData;

    imuData.accelerometer[0] = 1;
    imuData.accelerometer[1] = 2;
    imuData.accelerometer[2] = 3;
    imuData.gyroscope[0] = 4;
    imuData.gyroscope[1] = 5;
    imuData.gyroscope[2] = 6;

    imuData.magnetometer[0] = 7;
    imuData.magnetometer[1] = 8;
    imuData.magnetometer[2] = 9;

    float timeBefore = millis();

    fusion.movingAverageFilter(imuData);

    fusion.update(imuData, timestamp);
    float angles[3];
    fusion.getEulerAngles(angles);
    float timeElapsed = millis() - timeBefore;
    meanTime = meanTime + timeElapsed;
    //Serial.print("Iteracao n: ");
    //Serial.print(counter);
    //Serial.print(" | tempo: ");
    //Serial.print(timeElapsed);
    //Serial.print(" ms");
    //Serial.println();

    
    counter++;
  } else {
    Serial.println("Fim da execucao.");

    meanTime = meanTime / 10000;
    Serial.println("Média de tempo: ");
    Serial.print(meanTime);
    Serial.print(" ms");
    while (1); // Para o loop ao atingir 10000 iterações
  }
}
