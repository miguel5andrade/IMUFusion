#include "IMUFusion.h"
#include <cmath>
#include <cstdint>
#include <iostream>

namespace IMUFusion
{

    IMUFusion::IMUFusion(FilterType type, float beta)
        : filterType_(type), beta_(beta), q_{1.0f, 0.0f, 0.0f, 0.0f}, lastTimestamp_(0), hasDirectEuler_(false), roll_(0.0f), pitch_(0.0f), yaw_(0.0f)
    {
        // Initialize Kalman filter matrices
        P_ = Eigen::MatrixXf::Identity(4, 4) * 0.1f;
        Q_ = Eigen::MatrixXf::Identity(4, 4) * 0.001f;
        RAccel_ = Eigen::MatrixXf::Identity(3, 3) * 0.1f;
        RMag_ = Eigen::MatrixXf::Identity(3, 3) * 0.1f;
    }

    void IMUFusion::update(const IMUData &data, uint64_t timestamp, bool hasMagnetometer)
    {
        float dt = (lastTimestamp_ > 0) ? (timestamp - lastTimestamp_) / 1000000.0f : 0.0f;
        lastTimestamp_ = timestamp;
        if (dt <= 0.0f)
            return;

        if (filterType_ == FilterType::MADGWICK)
        {
            hasDirectEuler_ = false;
            updateMadgwick(data, dt, hasMagnetometer);
        }
        else if (filterType_ == FilterType::KALMAN)
        {
            hasDirectEuler_ = false;
            updateKalman(data, dt, hasMagnetometer);
        }
        else
        {
            hasDirectEuler_ = true;
            updateComplementaryFilter(data, dt);
        }
    }

    void IMUFusion::updateComplementaryFilter(const IMUData &data, float dt)
    {

        // complementary filter takes into account the angle computed using the accelerometer and the gyroscope and averages them together
        float ax = data.accelerometer[0];
        float ay = data.accelerometer[1];
        float az = data.accelerometer[2];

        float gx = data.gyroscope[0];
        float gy = data.gyroscope[1];
        float gz = data.gyroscope[2];

        // Roll and Pitch from the accelerometer, its impossible to compute the yaw angle
        float roll_from_accel = std::atan2(ay, az);
        float pitch_from_accel = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        // Implement complementary filter with integration
        float alpha = 0.98f; // Weight factor for gyro data             //**********TODO, USER CAN CHANGE THIS VALUE

        // Create transformation matrix T(roll, pitch)
        float T11 = 1.0f;
        float T12 = std::sin(roll_) * std::tan(pitch_);
        float T13 = std::cos(roll_) * std::tan(pitch_);
        float T21 = 0.0f;
        float T22 = std::cos(roll_);
        float T23 = -std::sin(roll_);
        float T31 = 0.0f;
        float T32 = std::sin(roll_) / std::cos(pitch_);
        float T33 = std::cos(roll_) / std::cos(pitch_);

        // Apply transformation to angular velocities
        float roll_dot = T11 * gx + T12 * gy + T13 * gz;
        float pitch_dot = T21 * gx + T22 * gy + T23 * gz;
        float yaw_dot = T31 * gx + T32 * gy + T33 * gz;

        // Integrate gyroscope data for roll and pitch

        roll_ = alpha * (roll_ + roll_dot * dt) + (1.0f - alpha) * roll_from_accel;
        pitch_ = alpha * (pitch_ + pitch_dot * dt) + (1.0f - alpha) * pitch_from_accel;
        yaw_ += yaw_dot * dt;
    }

    void IMUFusion::updateKalman(const IMUData &data, float dt, bool hasMagnetometer)
    {
        // State vector: [q_w, q_x, q_y, q_z]
        Eigen::Vector4f x(q_[0], q_[1], q_[2], q_[3]);

        // 1. State Prediction using Gyroscope
        Eigen::Vector4f gyroDelta;
        float gx = data.gyroscope[0];
        float gy = data.gyroscope[1];
        float gz = data.gyroscope[2];

        // Compute quaternion derivative based on gyroscope data
        Eigen::Matrix4f F;
        F << 0.0f, -gx, -gy, -gz,
            gx, 0.0f, gz, -gy,
            gy, -gz, 0.0f, gx,
            gz, gy, -gx, 0.0f;
        F *= 0.5f; // Derivative factor for quaternion

        // Predict state
        x += F * x * dt;

        // Predict covariance
        P_ = P_ + Q_ * dt;

        // Normalize quaternion
        x.normalize();

        // 2. Accelerometer Measurement Update
        Eigen::Vector3f accel(data.accelerometer[0], data.accelerometer[1], data.accelerometer[2]);
        accel.normalize();

        // Gravity vector in quaternion terms
        Eigen::Vector3f hAccel;
        hAccel << 2.0f * (x(1) * x(3) - x(0) * x(2)),
            2.0f * (x(2) * x(3) + x(0) * x(1)),
            x(0) * x(0) - x(1) * x(1) - x(2) * x(2) + x(3) * x(3);

        // Jacobian for accelerometer
        Eigen::MatrixXf HAccel(3, 4);
        HAccel << -2 * x(2), 2 * x(3), -2 * x(0), 2 * x(1),
            2 * x(1), 2 * x(0), 2 * x(3), 2 * x(2),
            2 * x(0), -2 * x(1), -2 * x(2), 2 * x(3);

        // Update step with accelerometer
        Eigen::MatrixXf KAccel = P_ * HAccel.transpose() * (HAccel * P_ * HAccel.transpose() + RAccel_).inverse();
        x += KAccel * (accel - hAccel);
        P_ = (Eigen::MatrixXf::Identity(4, 4) - KAccel * HAccel) * P_;

        // 3. Magnetometer Update (if available)
        if (hasMagnetometer)
        {
            Eigen::Vector3f mag(data.magnetometer[0], data.magnetometer[1], data.magnetometer[2]);
            mag.normalize();

            // Magnetic field vector in quaternion terms
            Eigen::Vector3f hMag;
            hMag << 2.0f * (x(1) * x(3) - x(0) * x(2)),
                2.0f * (x(2) * x(3) + x(0) * x(1)),
                x(0) * x(0) - x(1) * x(1) - x(2) * x(2) + x(3) * x(3);

            // Jacobian for magnetometer
            Eigen::MatrixXf HMag(3, 4);
            HMag << -2 * x(2), 2 * x(3), -2 * x(0), 2 * x(1),
                2 * x(1), 2 * x(0), 2 * x(3), 2 * x(2),
                2 * x(0), -2 * x(1), -2 * x(2), 2 * x(3);

            // Update step with magnetometer
            Eigen::MatrixXf KMag = P_ * HMag.transpose() * (HMag * P_ * HMag.transpose() + RMag_).inverse();
            x += KMag * (mag - hMag);
            P_ = (Eigen::MatrixXf::Identity(4, 4) - KMag * HMag) * P_;
        }

        // Normalize quaternion
        x.normalize();

        // Update quaternion
        q_[0] = x(0);
        q_[1] = x(1);
        q_[2] = x(2);
        q_[3] = x(3);
    }
    void IMUFusion::updateMadgwick(const IMUData &data, float dt, bool hasMagnetometer)
    {
        if (dt <= 0.0f)
            return;

        // Extrair dados crus
        auto accel = data.accelerometer; // ax, ay, az
        auto gyro = data.gyroscope;      // gx, gy, gz
        auto mag = data.magnetometer;    // mx, my, mz

        // Normalizar vetores de aceleração e magnétometro (quando houver)
        normalize(accel);
        if (hasMagnetometer)
        {
            normalize(mag);
        }

        // Renomear componentes do quaternion para simplificar leitura
        float q1 = q_[0]; // w
        float q2 = q_[1]; // x
        float q3 = q_[2]; // y
        float q4 = q_[3]; // z

        // Passo 1: derivada do quaternion apenas pela rotação giroscópica
        // qDot = 0.5 * q ⊗ gyro (em rad/s)
        float qDot1 = 0.5f * (-q2 * gyro[0] - q3 * gyro[1] - q4 * gyro[2]);
        float qDot2 = 0.5f * (q1 * gyro[0] + q3 * gyro[2] - q4 * gyro[1]);
        float qDot3 = 0.5f * (q1 * gyro[1] - q2 * gyro[2] + q4 * gyro[0]);
        float qDot4 = 0.5f * (q1 * gyro[2] + q2 * gyro[1] - q3 * gyro[0]);

        // Passo 2: computar função-erro do acelerómetro (gravidade)
        // Equações padrão do Madgwick, onde (0,0,1) é a gravidade de referência no referencial inercial.
        // A saída de fA será um vector de 3 componentes (f1, f2, f3).
        float f1 = 2.0f * (q2 * q4 - q1 * q3) - accel[0];        // erro em x
        float f2 = 2.0f * (q1 * q2 + q3 * q4) - accel[1];        // erro em y
        float f3 = 2.0f * (0.5f - q2 * q2 - q3 * q3) - accel[2]; // erro em z

        // Gradiente do erro em relação ao quaternion (parcial derivada de fA)
        // Referência: Madgwick paper ou FusionAhrsMadgwickUpdate na x-io
        float dF_dq1 = 2.0f * (-q3) * f1 + 2.0f * (q2)*f2 + 2.0f * (-0.0f) * f3;    // ~deriv wrt q1
        float dF_dq2 = 2.0f * (q4)*f1 + 2.0f * (q1)*f2 + 2.0f * (-2.0f * q2) * f3;  // ~deriv wrt q2
        float dF_dq3 = -2.0f * (q1)*f1 + 2.0f * (q4)*f2 + 2.0f * (-2.0f * q3) * f3; // ~deriv wrt q3
        float dF_dq4 = 2.0f * (q2)*f1 + 2.0f * (q3)*f2 + 2.0f * (-0.0f) * f3;       // ~deriv wrt q4

        // Passo 3: se houver magnetómetro válido, adicionar função-erro do magnetómetro (f4, f5, f6)
        if (hasMagnetometer)
        {
            // Calcular o campo magnético de referência projetado no referencial do quaternion
            // hx, hy são as componentes horizontais, bx é o módulo horizontal, bz a componente vertical
            float hx = 2.0f * (mag[0] * (0.5f - q3 * q3 - q4 * q4) + mag[1] * (q2 * q3 - q1 * q4) + mag[2] * (q2 * q4 + q1 * q3));
            float hy = 2.0f * (mag[0] * (q2 * q3 + q1 * q4) + mag[1] * (0.5f - q2 * q2 - q4 * q4) + mag[2] * (q3 * q4 - q1 * q2));
            float bx = std::sqrt(hx * hx + hy * hy);
            float bz = 2.0f * (mag[0] * (q2 * q4 - q1 * q3) + mag[1] * (q3 * q4 + q1 * q2) + mag[2] * (0.5f - q2 * q2 - q3 * q3));

            // Função-erro do magnetómetro (comparando campo medido vs. campo esperado [bx, 0, bz])
            float f4 = 2.0f * (bx * (0.5f - q3 * q3 - q4 * q4) + bz * (q2 * q4 - q1 * q3)) - mag[0];
            float f5 = 2.0f * (bx * (q2 * q3 - q1 * q4) + bz * (q1 * q2 + q3 * q4)) - mag[1];
            float f6 = 2.0f * (bx * (q1 * q3 + q2 * q4) + bz * (0.5f - q2 * q2 - q3 * q3)) - mag[2];

            // Gradiente do erro do magnetómetro
            // Derivadas parciais f4, f5, f6 wrt q1,q2,q3,q4 (ver eq. no Madgwick ou FusionAhrsMadgwickUpdate)
            float dM_dq1 = 2.0f * (-bz * q3) * f4 + 2.0f * (-bx * q4 + bz * q2) * f5 + 2.0f * (bx * q3) * f6;
            float dM_dq2 = 2.0f * (bz * q4) * f4 + 2.0f * (bx * q3 + bz * q1) * f5 + 2.0f * (bx * q4 - 2.0f * bz * q2) * f6;
            float dM_dq3 = 2.0f * (-bx * q4 - bz * q1) * f4 + 2.0f * (bx * q2) * f5 + 2.0f * (bx * q1 - 2.0f * bz * q3) * f6;
            float dM_dq4 = 2.0f * (bx * q2 - bz * q3) * f4 + 2.0f * (-bx * q1) * f5 + 2.0f * (bx * q2) * f6;

            // Somar magnetómetro ao gradiente total
            dF_dq1 += dM_dq1;
            dF_dq2 += dM_dq2;
            dF_dq3 += dM_dq3;
            dF_dq4 += dM_dq4;
        }

        // Passo 4: normalizar gradiente para que a correcção não expluda numericamente
        float normGrad = std::sqrt(dF_dq1 * dF_dq1 + dF_dq2 * dF_dq2 + dF_dq3 * dF_dq3 + dF_dq4 * dF_dq4);
        if (normGrad > 1e-9f)
        {
            dF_dq1 /= normGrad;
            dF_dq2 /= normGrad;
            dF_dq3 /= normGrad;
            dF_dq4 /= normGrad;
        }

        // Passo 5: atualizar a derivada do quaternion com o termo de correcção
        qDot1 -= beta_ * dF_dq1;
        qDot2 -= beta_ * dF_dq2;
        qDot3 -= beta_ * dF_dq3;
        qDot4 -= beta_ * dF_dq4;

        // Passo 6: integrar no tempo
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
        q4 += qDot4 * dt;

        // Passo 7: normalizar o quaternion final
        float normQ = std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        if (normQ > 1e-9f)
        {
            q1 /= normQ;
            q2 /= normQ;
            q3 /= normQ;
            q4 /= normQ;
        }

        // Guardar de volta
        q_[0] = q1;
        q_[1] = q2;
        q_[2] = q3;
        q_[3] = q4;
    }

    std::array<float, 4> IMUFusion::getQuaternion() const
    {
        return q_;
    }

    std::array<float, 3> IMUFusion::getEulerAngles() const
    {

        if (hasDirectEuler_)
        {
            return {roll_, pitch_, yaw_};
        }

        // Quaternion: q_[0] = w, q_[1] = x, q_[2] = y, q_[3] = z
        const float &w = q_[0];
        const float &x = q_[1];
        const float &y = q_[2];
        const float &z = q_[3];

        // Converter quaternion em ângulos de Euler (Roll, Pitch, Yaw)
        float roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
        float pitch = std::asin(2.0f * (w * y - z * x));
        float yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

        return {roll, pitch, yaw};
    }

    void IMUFusion::normalize(std::array<float, 3> &v) const
    {
        float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (norm > 1e-9f)
        {
            v[0] /= norm;
            v[1] /= norm;
            v[2] /= norm;
        }
    }

    float IMUFusion::invSqrt(float x) const
    {
        return 1.0f / std::sqrt(x); // Alternatively, implement fast inverse square root
    }

    void IMUFusion::setBeta(float beta)
    {
        beta_ = beta;
    }

    void IMUFusion::setProcessNoise(float q)
    {
        Q_ = Eigen::MatrixXf::Identity(4, 4) * q;
    }

    void IMUFusion::setMeasurementNoise(float rAccel, float rMag)
    {
        RAccel_ = Eigen::MatrixXf::Identity(3, 3) * rAccel;
        RMag_ = Eigen::MatrixXf::Identity(3, 3) * rMag;
    }

    void IMUFusion::normalizeQuaternion()
    {
        float norm = std::sqrt(q_[0] * q_[0] + q_[1] * q_[1] +
                               q_[2] * q_[2] + q_[3] * q_[3]);
        for (int i = 0; i < 4; i++)
        {
            q_[i] /= norm;
        }
    }
} // namespace IMUFusion
