// OBSERVAÇÃO: O código ainda não está em sua versão final.

#include "mbed.h"
#include <cmath>

// Definindo os pinos I2C
I2C i2c(PB_6, PB_7); // SDA e SCL

// Endereço do sensor MPU-6050
#define MPU_ADDR 0x68 << 1  

// Registradores do MPU-6050 (Endereço no mapa de registro do MPU-6050)
#define SMPLRT_DIV 0x19
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48


// Função para ler dados do MPU-6050
int read_mpu_register(int reg) {
    char data[1];
    i2c.write(MPU_ADDR, (char *)&reg, 1, true);
    i2c.read(MPU_ADDR, data, 1);
    return (int)data[0];
}

// Função responsável por sempre manter o valor de Yaw atual no intervalo [-pi, pi]
float normalize_angle(float angle, float PI){
    while (angle > PI){
        angle -= 2 * PI;
    }
    while (angle < -PI){
        angle += 2 * PI;
    }
    return angle;
}

// Inicializa o MPU, tirando-o do sleep mode
void initialize_mpu(){

    // Desativa o modo de suspensão
    char data[2];   
    data[0] = PWR_MGMT_1;
    data[1] = 0;
    i2c.write(MPU_ADDR, data, 2);

    // Giroscópio sample rate
    data[0] = SMPLRT_DIV;
    data[1] = 39; // ACELERÔMETRO = 4 --> ideia é dar 200Hz, visando um deltaTime de 5ms.
    i2c.write(MPU_ADDR, data, 2);
    
}

// Função para ler os valores de giroscópio
void read_gyroscope(float &gx, float &gy, float &gz) {
    gx = (read_mpu_register(GYRO_XOUT_H) << 8) | read_mpu_register(GYRO_XOUT_L);
    gy = (read_mpu_register(GYRO_YOUT_H) << 8) | read_mpu_register(GYRO_YOUT_L);
    gz = (read_mpu_register(GYRO_ZOUT_H) << 8) | read_mpu_register(GYRO_ZOUT_L);
}

// Função para ler os valores dos acelerômetros
void read_acceleration(float &ax, float &ay, float &az) {
    ax = (read_mpu_register(ACCEL_XOUT_H) << 8) | read_mpu_register(ACCEL_XOUT_L);
    ay = (read_mpu_register(ACCEL_YOUT_H) << 8) | read_mpu_register(ACCEL_YOUT_L);
    az = (read_mpu_register(ACCEL_ZOUT_H) << 8) | read_mpu_register(ACCEL_ZOUT_L);
}

int main() {
    initialize_mpu();
    i2c.frequency(400000); // Frequência setada para 400kHz 
    
    // Variáveis principais de uso
    int frames = 3500;
    float alpha = 0.97; // Porcentagem de utilização do valor do giroscópio para o filtro complementar
    float gyro_x_cru, gyro_y_cru, gyro_z_cru;
    float acc_x_cru, acc_y_cru, acc_z_cru; 
    float gyro_x, gyro_y, gyro_z;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;
    float acc_x, acc_y, acc_z;
    float acc_x_offset, acc_y_offset;
    float thetha_X_gyro, thetha_Y_gyro, thetha_Z_gyro;
    float thetha_X_acc, thetha_Y_acc;
    float currentRoll = 0.0, currentPitch = 0.0, currentYaw = 0.0; 

    const float PI = 3.1415926535;
    bool tombo; // Indica se o carrinho tombou

    // Cálculo offset
    for (int i = 0; i < frames; i++){
        read_gyroscope(gyro_x_cru, gyro_y_cru, gyro_z_cru);

        // Full scale, +/-2000°/s
        gyro_x_offset += gyro_x_cru / 16.4;
        gyro_y_offset += gyro_y_cru / 16.4;
        gyro_z_offset += gyro_z_cru / 16.4;

        // Full scale, +/-16g
        acc_x_offset += acc_x_cru / 2048.0;
        acc_y_offset += acc_y_cru / 2048.0;
        }
    
    // Média dos valores
    gyro_x_offset /= frames;
    gyro_y_offset /= frames;
    gyro_z_offset /= frames;

    acc_x_offset /= frames;
    acc_y_offset /= frames;
    
    while (true) {
        read_gyroscope(gyro_x_cru, gyro_y_cru, gyro_z_cru); 
        read_acceleration(acc_x_cru, acc_y_cru, acc_z_cru);

        // Calibração do giroscópio e acelerômetro por offset
        gyro_x = (gyro_x_cru / 16.4) - gyro_x_offset;
        gyro_y = (gyro_y_cru / 16.4) - gyro_y_offset;
        gyro_z = (gyro_z_cru / 16.4) - gyro_z_offset;
        
        acc_x = (acc_x_cru / 2048.0) - acc_x_offset; 
        acc_y = (acc_y_cru / 2048.0) - acc_y_offset;
        
        // °/s para rad/s
        gyro_x = gyro_x * (PI / 180.0);
        gyro_y = gyro_y * (PI / 180.0);
        gyro_z = gyro_z * (PI / 180.0); 

        // Cálculo dos ângulos 

        thetha_X_gyro = normalize_angle(gyro_x * 0.005, PI); 
        thetha_Y_gyro = normalize_angle(gyro_y * 0.005, PI);
        thetha_Z_gyro = normalize_angle(gyro_z * 0.005, PI);
        
        thetha_X_acc = normalize_angle(atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)), PI);
        thetha_Y_acc = normalize_angle(atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)), PI);

        // Ângulos do momento 
        currentYaw = thetha_Z_gyro + currentYaw;

        // Filtro complementar para valores de roll e pitch, visando reduzir a deriva.
        currentRoll = alpha * (thetha_X_gyro + currentRoll) + (1 - alpha) * thetha_X_acc; 
        currentPitch = alpha * (thetha_Y_gyro + currentPitch) + (1 - alpha) * thetha_Y_acc;
        
        // Exibindo os valores lidos 
        printf("velocidadeX = %f rad/s, velocidadeY = %f rad/s, velocidadeZ = %f rad/s\n", gyro_x, gyro_y, gyro_z);
        printf("anguloX = %f rad, anguloY = %f rad, anguloZ = %f rad \n", currentRoll, currentPitch, currentPitch);

        // Verificando se o robô tombou para sinalização 
        if ((currentRoll > (45 * (PI / 180))) || (currentPitch > (45 * (PI / 180)))){ 
            tombo = true;
            // Acende led vermelho de sinalização
            printf("Alerta! O robozinho tombou!\n");
        } else {
            tombo = false;
        }  

        ThisThread::sleep_for(5ms); // Intervalo de 5ms entre cada leitura.
    }
}
