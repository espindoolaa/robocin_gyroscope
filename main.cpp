#include "mbed.h"
#include <cmath>

// Definindo os pinos I2C
I2C i2c(PB_7, PB_6); // SCL e SDA

// Criação do objeto timer
Timer temporizador;

// Endereço do sensor MPU-6050
#define MPU_ADDR 0x68 << 1  // O endereço é 0x68, mas deve ser deslocado para a esquerda

// Registradores do MPU-6050 (endereço no mapa de registro do MPU-6050)
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

// função para ler dados do MPU-6050
int read_mpu_register(int reg) {
    char data[1];
    // i2c.frequency(int hz) ver isso com Elisson
    i2c.write(MPU_ADDR, (char *)&reg, 1, true);
    i2c.read(MPU_ADDR, data, 1);
    return (int)data[0];
}

// função para ler os valores de giroscópio e dos acelerômetros
void read_gyroscope(float &gx, float &gy, float &gz) {
    gx = (read_mpu_register(GYRO_XOUT_H) << 8) | read_mpu_register(GYRO_XOUT_L);
    gy = (read_mpu_register(GYRO_YOUT_H) << 8) | read_mpu_register(GYRO_YOUT_L);
    gz = (read_mpu_register(GYRO_ZOUT_H) << 8) | read_mpu_register(GYRO_ZOUT_L);
}

void read_acceleration(float &ax, float &ay, float &az) {
    ax = (read_mpu_register(ACCEL_XOUT_H) << 8) | read_mpu_register(ACCEL_XOUT_L);
    ay = (read_mpu_register(ACCEL_YOUT_H) << 8) | read_mpu_register(ACCEL_YOUT_L);
    az = (read_mpu_register(ACCEL_ZOUT_H) << 8) | read_mpu_register(ACCEL_ZOUT_L);
}


int main() {
    // inicializando o MPU-6050
    char data[2] = {PWR_MGMT_1, 0};  // acordando o sensor
    i2c.write(MPU_ADDR, data, 2);
    
    int frames = 3500;
    float gyro_x_cru, gyro_y_cru, gyro_z_cru;
    float acc_x_cru, acc_y_cru, acc_z_cru; 
    float gyro_x, gyro_y, gyro_z;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;
    float acc_x, acc_y, acc_z;
    float acc_x_offset, acc_y_offset, acc_z_offset;
    float thetha_X_gyro, thetha_Y_gyro, thetha_Z_gyro;
    float thetha_X_acc, thetha_Y_acc, thetha_Z_acc;

    float currentRoll = 0.0, currentPitch = 0.0, currentYaw = 0.0; // o roll e pitch depois serão submetidos ao Kalman's Filter 

    float deltaTempo = 0.0, tempoAtual = 0.0, tempoInicial = 0.0;

    const float PI = 3.1415926535;
    bool tombo; // indica se o carrinho tombou

    temporizador.start(); // inicia o temporizador  

    // CÁLCULO OFFSET
    for (int i = 0; i < frames; i++){
        read_gyroscope(gyro_x_cru, gyro_y_cru, gyro_z_cru);

        gyro_x_offset += gyro_x_cru / 131.0;
        gyro_y_offset += gyro_y_cru / 131.0;
        gyro_z_offset += gyro_z_cru / 131.0;

        acc_x_offset += acc_x_cru / 16.384;
        acc_y_offset += acc_y_cru / 16.384;
        acc_z_offset += acc_z_cru / 16.384;
        }
    
    // MÉDIA DOS VALORES
    gyro_x_offset = gyro_x_cru / frames;
    gyro_y_offset = gyro_y_cru / frames;
    gyro_z_offset = gyro_z_cru / frames;

    acc_x_offset = acc_x_cru / frames;
    acc_y_offset = acc_y_cru / frames;
    acc_z_offset = acc_z_cru / frames;
    
    tempoInicial = chrono::duration_cast<chrono::seconds>(temporizador.elapsed_time()).count();
    while (true) {
        read_gyroscope(gyro_x_cru, gyro_y_cru, gyro_z_cru); 
        read_acceleration(acc_x_cru, acc_y_cru, acc_z_cru);

        // CALIBRAÇÃO DO GIROSCÓPIO E ACELERÔMETROS POR OFFSET
        gyro_x = (gyro_x_cru / 131.0) - gyro_x_offset;
        gyro_y = (gyro_y_cru / 131.0) - gyro_y_offset;
        gyro_z = (gyro_z_cru / 131.0) - gyro_z_offset;
        
        acc_x = (acc_x_cru / 131.0) - acc_x_offset; 
        acc_y = (acc_y_cru / 131.0) - acc_y_offset;
        acc_z = (acc_z_cru / 131.0) - acc_z_offset;
        
        // °/s PARA RAD/S
        gyro_x = gyro_x * (PI / 180.0);
        gyro_y = gyro_y * (PI / 180.0);
        gyro_z = gyro_z * (PI / 180.0); 

        
        tempoAtual = chrono::duration_cast<chrono::seconds>(temporizador.elapsed_time()).count();
        deltaTempo = tempoAtual - tempoInicial;
        tempoInicial = tempoAtual;

        // CÁLCULO DOS ÂNGULOS (INTEGRAÇÃO)

        thetha_X_gyro = gyro_x * deltaTempo; 
        thetha_Y_gyro = gyro_y * deltaTempo;
        thetha_Z_gyro = gyro_z * deltaTempo;
        
        thetha_X_acc = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z));
        thetha_Y_acc = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z));

        // ANGULOS DO MOMENTO
        
        currentRoll = thetha_X_gyro + currentRoll;
        currentPitch = thetha_Y_gyro + currentPitch;
        currentYaw = thetha_Z_gyro + currentYaw;
        

        // exibindo os valores lidos 
        printf("velocidadeX = %f rad/s, velocidadeY = %f rad/s, velocidadeZ = %f rad/s\n", gyro_x, gyro_y, gyro_z);
        printf("anguloX = %f rad, anguloY = %f rad, anguloZ = %f rad \n", thetha_X_gyro, thetha_Y_gyro, thetha_Z_gyro);

        if ((currentRoll > (45 * (PI / 180))) || (currentPitch > (45 * (PI / 180)))){ 
            tombo = true;
            // acende led vermelho de sinalização
            printf("Alerta! O robozinho tombou!\n");
        } else {
            tombo = false;
        }  

        // atraso para leitura contínua
        wait_us(100000);  // 100ms
    }
}
