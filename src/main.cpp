// OBSERVAÇÃO: O código ainda não está em sua versão final.

#include "mbed.h"
#include <cmath>
#include <nRF24Communication.h>

// Definindo os pinos I2C
I2C i2c(PB_7, PB_6); // SDA e SCL

// Endereço do sensor MPU-6050
#define MPU_ADDR 0x68 << 1

// Registradores do MPU-6050 (Endereço no mapa de registro do MPU-6050)
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
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

nRF24Communication radio_recv(NRF_R_MOSI,
                              NRF_R_MISO,
                              NRF_R_SCK,
                              NRF_R_CE,
                              NRF_R_CSN,
                              NRF_R_VCC,
                              NetworkType::ssl,
                              RadioFunction::receiver);


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

// Inicializa o MPU, configurando-o
void initialize_mpu(){
    // Desativa o modo de suspensão
    char data[2];   
    data[0] = PWR_MGMT_1;
    data[1] = 0x00;
    i2c.write(MPU_ADDR, data, 2);

    // Configura o filtro passa-baixa digital (DLPF_CFG = 3)  
    data[0] = CONFIG_REG;
    data[1] = 0x03;
    i2c.write(MPU_ADDR, data, 2);

    // Taxa de amostragem 
    data[0] = SMPLRT_DIV;
    data[1] = 0x18; // ideia é dar 200Hz, visando um deltaTime de 5ms.
    i2c.write(MPU_ADDR, data, 2);

    // Configuração do fundo de escala do acelerômetro
    data[0] = GYRO_CONFIG_REG;
    data[1] = 0x18; // Valor para configurar ±2000 °/s
    i2c.write(MPU_ADDR, data, 2);

    // Configuração do fundo de escala do giroscópio
    data[0] = ACCEL_CONFIG_REG  ;
    data[1] = 0x18; // Valor para configurar ±16g
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
    // DigitalOut led_tombamento(PINO);   
    i2c.frequency(400000); // Frequência setada para 400kHz - Modo Turbo
    
    int frames = 3676; 
    float dt = 0.005;

    //----------------------
    // Variáveis dos filtros 
    //----------------------
    float alpha = 0.98; 
    float p = 4.0;  // Incerteza inicial do filtro
    float q = 0.01;  // Ruído do giroscópio (ajustável)
    float r = 9.0;   // Ruído do acelerômetro (ajustável)
    float k; // Ganho de Kalman 

    //-----------------------
    // Variáveis das medições
    //-----------------------
    float gyro_x_cru, gyro_y_cru, gyro_z_cru;
    float acc_x_cru, acc_y_cru, acc_z_cru; 
    float gyro_x, gyro_y, gyro_z;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;
    float acc_x, acc_y, acc_z;
    float acc_x_offset, acc_y_offset;
    float gyro_angle_x = 0.0, gyro_angle_y = 0.0, gyro_angle_z = 0.0;
    float thetha_x_acc, thetha_y_acc;
    float currentRoll = 0.0, currentPitch = 0.0, currentYaw = 0.0; 

    const float PI = 3.1415926535;


    // Cálculo offset
    for (int i = 0; i < frames; i++){
        read_gyroscope(gyro_x_cru, gyro_y_cru, gyro_z_cru);
        read_acceleration(acc_x_cru, acc_y_cru, acc_z_cru);

        // Full scale, +/-2000°/s
        gyro_x_offset += gyro_x_cru;
        gyro_y_offset += gyro_y_cru;
        gyro_z_offset += gyro_z_cru;

        // Full scale, +/-16g
        acc_x_offset += acc_x_cru;
        acc_y_offset += acc_y_cru;
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
        gyro_x = gyro_x_cru - gyro_x_offset;
        gyro_y = gyro_y_cru - gyro_y_offset;
        gyro_z = gyro_z_cru - gyro_z_offset;
        
        acc_x = acc_x_cru - acc_x_offset; 
        acc_y = acc_y_cru - acc_y_offset;
        
        // °/s para rad/s
        gyro_x = gyro_x * (PI / 180.0);
        gyro_y = gyro_y * (PI / 180.0);
        gyro_z = gyro_z * (PI / 180.0); 

        // Cálculo dos ângulos 
        thetha_x_acc = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z));
        thetha_y_acc = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z));

        // Ângulos do momento 
        gyro_angle_x = gyro_angle_x + gyro_x * dt;
        gyro_angle_y = gyro_angle_y + gyro_y * dt;
        currentYaw = alpha * (currentYaw + gyro_z * dt) + (1 - alpha) * currentYaw;

        // Kalman gain 
        k = p / (p + r);

        // Kalman filter
        currentRoll = normalize_angle((gyro_angle_x + k * (thetha_x_acc - gyro_angle_x)), PI);
        currentPitch = normalize_angle((gyro_angle_y + k * (thetha_y_acc - gyro_angle_y)), PI);
        
        // Atualização da incerteza
        p = (1 - k) * p + q;
        
        // Exibindo os valores lidos 
        printf("velocidadeX = %f rad/s, velocidadeY = %f rad/s, velocidadeZ = %f rad/s\n", gyro_x, gyro_y, gyro_z);
        printf("anguloX = %f rad, anguloY = %f rad, anguloZ = %f rad \n", currentRoll, currentPitch, currentPitch);
        
        //----------------------------------
        // Sistema de análise de tombamento
        //----------------------------------

        if ((currentRoll > (45 * (PI / 180))) || (currentPitch > (45 * (PI / 180)))){  
            while ((currentRoll > (45 * (PI / 180))) || (currentPitch > (45 * (PI / 180)))){
                // led_tombamento = 1;
                printf("Alerta! O robozinho tombou!\n");
                
            }
            // led_tombamento = 0;
        }

        //----------------------------------------------------------
        // Verificação se o jogo está parado ou não para recalibrar
        //----------------------------------------------------------

        if (radio_recv.getGameState() == refereeCommand::halt){
            if ((currentRoll < 45.0) && (currentPitch < 45.0)){
                currentRoll = 0.0;
                currentPitch = 0.0;
                p = 4.0;
                q = 0.01;
                r = 9;
            }
        }

        ThisThread::sleep_for(5ms); // Intervalo de 5ms entre cada leitura 
    }
}
