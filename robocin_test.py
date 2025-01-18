frames = 3676
currentYaw = 0.0 
PI = 3.141592653589793

def normalize_angle(angle):
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle

def calibrando(frames):
    gyro_z_offset = 0.0
    for i in range(0, frames):
        local_w, odometry_theta = input().split(",")
        local_w = float(local_w)
        odometry_theta = float(odometry_theta)
        gyro_z_offset += local_w 
    gyro_z_offset /= frames
    return gyro_z_offset


gyro_z_offset = calibrando(frames)
for i in range(0, 6324):
    gyro_z, odometry_theta = input().split(",")
    gyro_z = float(gyro_z)
    odometry_theta = float(odometry_theta)

    # CALIBRAÇÃO DO GIROSCÓPIO E TRANSFORMAÇÃO PARA RAD/S
    gyro_z = (gyro_z - gyro_z_offset) * (PI / 180.0)

    # ANGULO ENCONTRADO
    thetha_z_gyro = gyro_z * 0.005 
    
    # ANGULOS DO MOMENTO
    
    currentYaw = normalize_angle(thetha_z_gyro + currentYaw)
    erroAcumulado = odometry_theta - currentYaw

    # EXIBIÇÃO DOS VALORES OBTIDOS 
    print(f"yaw_dataset = {odometry_theta} rad/s, yaw_codigo = {currentYaw} rad/s")
    print(f"erro acumulado = {erroAcumulado}")
