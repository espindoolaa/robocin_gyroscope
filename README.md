# Gyroscope Project

Esse projeto tem por objetivo implementar num sensor MPU-6050, associado à um microcontrolador STM32F767ZI, a leitura do seu giroscópio a fim de calcular o deslocamento e velocidade angular do carrinho da modalidade SSL.

## Abordagens definidas

- Framework: Mbed 
- Sensor: MPU6050 
- Board: NUCLEO-F767ZI
- Calibração por Offset
- Aplicação do filtro de Kalman para Pitch e Roll
- Uso do filtro complementar para o Yaw, dando-o maior estabilidade.
- Validação em código Python e em construção presencial, utilizando Arduino.

## Estruturação do projeto 

![image](https://github.com/user-attachments/assets/16b88dc4-e49c-4590-b066-b056b8460e70)
