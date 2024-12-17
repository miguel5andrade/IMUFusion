#include "IMUFusion.h"
#include <cmath>
#include <cstdint>
#include <iostream>

namespace IMUFusion
{

    IMUFusion::IMUFusion(FilterType type, float beta)
        : filterType_(type), beta_(beta), q_{1.0f, 0.0f, 0.0f, 0.0f}, lastTimestamp_(0), hasDirectEuler_(false), roll_(0.0f), pitch_(0.0f), yaw_(0.0f)
    {
        // Inicializa matriz de covariância P_ (6x6) e Q_ (6x6), RAccel_ (2x2), e state_ (6x1).
        for (int i = 0; i < 6; i++)
        {
            state_[i] = 0.0f; // Estado inicial zero
            for (int j = 0; j < 6; j++)
            {
                P_[i][j] = 0.0f;
                Q_[i][j] = 0.0f;
            }
        }
        // Covariância inicial P_ como identidade * 0.1f
        for (int i = 0; i < 6; i++)
        {
            P_[i][i] = 0.1f;
        }
        // Q_ como identidade * 0.001f
        for (int i = 0; i < 6; i++)
        {
            Q_[i][i] = 0.001f;
        }
        // RAccel_ (2x2) assume ruído do acelerómetro p/ roll e pitch
        RAccel_[0][0] = 0.1f;
        RAccel_[0][1] = 0.0f;
        RAccel_[1][0] = 0.0f;
        RAccel_[1][1] = 0.1f;
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
            hasDirectEuler_ = true;
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
        // Inputs:
        float ax = data.accelerometer[0];
        float ay = data.accelerometer[1];
        float az = data.accelerometer[2];

        float gx = data.gyroscope[0];
        float gy = data.gyroscope[1];
        float gz = data.gyroscope[2];

        // ---- 1) PREDICT ----
        // Matrizes F e P preditas
        // Vamos criar a matriz F como identidade + dt nas posições adequadas.
        float F[6][6];
        // Inicializar F como identidade
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                F[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        // roll -> + roll_rate * dt
        F[0][3] = dt;
        // pitch -> + pitch_rate * dt
        F[1][4] = dt;
        // yaw -> + yaw_rate * dt
        F[2][5] = dt;

        // Precisamos primeiro transformar gx, gy, gz do body frame para o inertial (roll_, pitch_ do estado atual)
        // Vamos supor que a estimativa atual de roll e pitch é state_[0] e state_[1].
        float roll_est = state_[0];
        float pitch_est = state_[1];

        float T11 = 1.0f;
        float T12 = std::sin(roll_est) * std::tan(pitch_est);
        float T13 = std::cos(roll_est) * std::tan(pitch_est);
        float T21 = 0.0f;
        float T22 = std::cos(roll_est);
        float T23 = -std::sin(roll_est);
        float T31 = 0.0f;
        float T32 = std::sin(roll_est) / std::cos(pitch_est);
        float T33 = std::cos(roll_est) / std::cos(pitch_est);

        float roll_dot = T11 * gx + T12 * gy + T13 * gz;
        float pitch_dot = T21 * gx + T22 * gy + T23 * gz;
        float yaw_dot = T31 * gx + T32 * gy + T33 * gz;

        // Vetor xPredict
        float xPredict[6];
        for (int i = 0; i < 6; i++)
        {
            xPredict[i] = state_[i];
        }
        // roll, pitch, yaw
        xPredict[0] += state_[3] * dt; // roll += roll_rate*dt
        xPredict[1] += state_[4] * dt; // pitch += pitch_rate*dt
        xPredict[2] += state_[5] * dt; // yaw += yaw_rate*dt

        // roll_rate, pitch_rate, yaw_rate passam a ser roll_dot, pitch_dot, yaw_dot
        xPredict[3] = roll_dot;
        xPredict[4] = pitch_dot;
        xPredict[5] = yaw_dot;

        // P = F * P * F^T + Q
        float tempFP[6][6];
        matMul66(F, P_, tempFP); // tempFP = F * P
        float F_T[6][6];
        matTrans66(F, F_T); // F^T
        float newP[6][6];
        // newP = tempFP * F^T
        float temp[6][6];
        matMul66(tempFP, F_T, temp);

        // Adicionar Q_
        matAdd66(temp, Q_, newP);

        // ---- 2) UPDATE ----
        // Correção via acelerómetro: medimos roll, pitch

        float roll_meas = std::atan2(ay, az);
        float pitch_meas = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        float z[2] = {roll_meas, pitch_meas}; // Medição

        // h(x) = [roll, pitch] => [xPredict[0], xPredict[1]]
        float h[2] = {xPredict[0], xPredict[1]};

        // Inovação y = z - h
        float y[2] = {z[0] - h[0], z[1] - h[1]};

        // Matriz H(2x6): extrai roll e pitch do estado
        // H = [[1,0,0,0,0,0],
        //      [0,1,0,0,0,0]]
        float H[2][6];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                H[i][j] = 0.0f;
            }
        }
        H[0][0] = 1.0f; // roll
        H[1][1] = 1.0f; // pitch

        // S = H * newP * H^T + RAccel
        float HP[2][6];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 6; k++)
                {
                    soma += H[i][k] * newP[k][j];
                }
                HP[i][j] = soma;
            }
        }
        float S[2][2];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 6; k++)
                {
                    // H^T é H transposta => H[j][k] = H^T[k][j]
                    soma += HP[i][k] * H[j][k];
                }
                S[i][j] = soma + RAccel_[i][j];
            }
        }

        // Inverter S (2x2)
        float S_inv[2][2];
        if (!invert2x2(S, S_inv))
        {
            // Se não for invertível, abortar
            // mas normalmente não deve acontecer
            for (int i = 0; i < 6; i++)
            {
                state_[i] = xPredict[i];
            }
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    P_[i][j] = newP[i][j];
                }
            }
            return;
        }

        // K = newP * H^T * S_inv => dimension(6x2)
        // primeiro newP * H^T
        float PHt[6][2];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 6; k++)
                {
                    // H^T => H[j][k]
                    soma += newP[i][k] * H[j][k];
                }
                PHt[i][j] = soma;
            }
        }
        float K[6][2];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 2; k++)
                {
                    soma += PHt[i][k] * S_inv[k][j];
                }
                K[i][j] = soma;
            }
        }

        // Atualizar estado: x = xPredict + K*y
        float xUpdated[6];
        for (int i = 0; i < 6; i++)
        {
            float soma = xPredict[i];
            for (int j = 0; j < 2; j++)
            {
                soma += K[i][j] * y[j];
            }
            xUpdated[i] = soma;
        }

        // P = (I - K*H)*newP
        float KH[6][6];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 2; k++)
                {
                    soma += K[i][k] * H[k][j];
                }
                KH[i][j] = soma;
            }
        }

        float IminusKH[6][6];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i == j)
                    IminusKH[i][j] = 1.0f - KH[i][j];
                else
                    IminusKH[i][j] = 0.0f - KH[i][j];
            }
        }

        float PUpdated[6][6];
        matMul66(IminusKH, newP, PUpdated);

        // Guardar estado e covariância
        for (int i = 0; i < 6; i++)
        {
            state_[i] = xUpdated[i];
        }
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                P_[i][j] = PUpdated[i][j];
            }
        }

        // Por fim, atualizamos roll_, pitch_, yaw_ (para quem quiser ler via getEulerAngles se hasDirectEuler_ = false)
        roll_ = state_[0];
        pitch_ = state_[1];
        yaw_ = state_[2];
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
        // Ajusta Q_ (6x6)
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i == j)
                    Q_[i][j] = q;
                else
                    Q_[i][j] = 0.0f;
            }
        }
    }

    void IMUFusion::setMeasurementNoise(float rAccel, float rMag)
    {
        // RAccel_ (2x2)
        RAccel_[0][0] = rAccel;
        RAccel_[1][1] = rAccel;
        RAccel_[0][1] = 0.0f;
        RAccel_[1][0] = 0.0f;
        // magnetómetro não está implementado neste Kalman simplificado
        // mas poderíamos ter RMag_ e etc.
        (void)rMag; // ignora por ora
    }

    // ------------------- Funções auxiliares ---------------------

    // Multiplicação C = A * B (6x6)
    void IMUFusion::matMul66(const float A[6][6], const float B[6][6], float C[6][6])
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                float soma = 0.0f;
                for (int k = 0; k < 6; k++)
                {
                    soma += A[i][k] * B[k][j];
                }
                C[i][j] = soma;
            }
        }
    }

    // Soma C = A + B (6x6)
    void IMUFusion::matAdd66(const float A[6][6], const float B[6][6], float C[6][6])
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
    }

    // Transposta At = A^T (6x6)
    void IMUFusion::matTrans66(const float A[6][6], float At[6][6])
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                At[j][i] = A[i][j];
            }
        }
    }

    // Multiplicação out = A * v (6x6 * 6x1)
    void IMUFusion::matMulVec6(const float A[6][6], const float v[6], float out[6])
    {
        for (int i = 0; i < 6; i++)
        {
            float soma = 0.0f;
            for (int j = 0; j < 6; j++)
            {
                soma += A[i][j] * v[j];
            }
            out[i] = soma;
        }
    }

    // Inverter 2x2
    bool IMUFusion::invert2x2(const float M[2][2], float Minv[2][2])
    {
        float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (std::fabs(det) < 1e-9f)
            return false;
        float invDet = 1.0f / det;
        Minv[0][0] = M[1][1] * invDet;
        Minv[0][1] = -M[0][1] * invDet;
        Minv[1][0] = -M[1][0] * invDet;
        Minv[1][1] = M[0][0] * invDet;
        return true;
    }
} // namespace IMUFusion
