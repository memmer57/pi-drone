import time
import smbus
import math
import statistics
import socket
import RPi.GPIO as GPIO
from multiprocessing import Process
import threading

#
#    0     1
#     #   #
#      ###
#      ###
#     #   #
#    3     2
#

emergency_stop = [0]

KP_roll = 0.004
KI_roll = 0.006
KD_roll = 0.005

KP_pitch = 0.004
KI_pitch = 0.006
KD_pitch = 0.005

KP_yaw = 0.004
KI_yaw = 0.006
KD_yaw = 0.005

GyroPrev = [0,0,0]
GamepadPrev = [0,0,0]

motor = [0,0,0,0]
motor_gpio = [38, 35, 13, 18]
motor_control = []

gamepad_output = [0,0,0,0]

Gyro = [0,0,0]
Accel = [0,0,0]

AccelWindowX = []
AccelWindowY = []
AccelWindowZ = []

GyroAngles = [0,0,0]
AccelAngles = [0,0]
Angles = [0,0,0]

#GyroOffset = [-4.43, 20.46 , -0.308]
#AccelOffset = [0.0075, -0.0368, -0.056]
GyroOffset = [0,0,0]
AccelOffset = [0,0,0]

rate_pid_integral = [0,0,0]
angle_pid_integral = [0,0,0]

GyroSensitivity = 32.8
AccelSensitivity = 4096

RAD_TO_DEG = 180 / math.pi
true = 0x01
false = 0x00
I2C_ADD_ICM20948 = 0x68
I2C_ADD_ICM20948_AK09916 = 0x0C
I2C_ADD_ICM20948_AK09916_READ = 0x80
I2C_ADD_ICM20948_AK09916_WRITE = 0x00
REG_ADD_WIA = 0x00
REG_VAL_WIA = 0xEA
REG_ADD_USER_CTRL = 0x03
REG_VAL_BIT_DMP_EN = 0x80
REG_VAL_BIT_FIFO_EN = 0x40
REG_VAL_BIT_I2C_MST_EN = 0x20
REG_VAL_BIT_I2C_IF_DIS = 0x10
REG_VAL_BIT_DMP_RST = 0x08
REG_VAL_BIT_DIAMOND_DMP_RST = 0x04
REG_ADD_PWR_MIGMT_1 = 0x06
REG_VAL_ALL_RGE_RESET = 0x80
REG_VAL_RUN_MODE = 0x01
REG_ADD_LP_CONFIG = 0x05
REG_ADD_PWR_MGMT_1 = 0x06
REG_ADD_PWR_MGMT_2 = 0x07
REG_ADD_ACCEL_XOUT_H = 0x2D
REG_ADD_ACCEL_XOUT_L = 0x2E
REG_ADD_ACCEL_YOUT_H = 0x2F
REG_ADD_ACCEL_YOUT_L = 0x30
REG_ADD_ACCEL_ZOUT_H = 0x31
REG_ADD_ACCEL_ZOUT_L = 0x32
REG_ADD_GYRO_XOUT_H = 0x33
REG_ADD_GYRO_XOUT_L = 0x34
REG_ADD_GYRO_YOUT_H = 0x35
REG_ADD_GYRO_YOUT_L = 0x36
REG_ADD_GYRO_ZOUT_H = 0x37
REG_ADD_GYRO_ZOUT_L = 0x38
REG_ADD_EXT_SENS_DATA_00 = 0x3B
REG_ADD_REG_BANK_SEL = 0x7F
REG_VAL_REG_BANK_0 = 0x00
REG_VAL_REG_BANK_1 = 0x10
REG_VAL_REG_BANK_2 = 0x20
REG_VAL_REG_BANK_3 = 0x30
REG_ADD_GYRO_SMPLRT_DIV = 0x00
REG_ADD_GYRO_CONFIG_1 = 0x01
REG_VAL_BIT_GYRO_DLPCFG_2 = 0x10
REG_VAL_BIT_GYRO_DLPCFG_4 = 0x20
REG_VAL_BIT_GYRO_DLPCFG_6 = 0x30
REG_VAL_BIT_GYRO_FS_250DPS = 0x00
REG_VAL_BIT_GYRO_FS_500DPS = 0x02
REG_VAL_BIT_GYRO_FS_1000DPS = 0x04
REG_VAL_BIT_GYRO_FS_2000DPS = 0x06
REG_VAL_BIT_GYRO_DLPF = 0x01
REG_ADD_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ADD_ACCEL_CONFIG = 0x14
REG_VAL_BIT_ACCEL_DLPCFG_2 = 0x10
REG_VAL_BIT_ACCEL_DLPCFG_4 = 0x20
REG_VAL_BIT_ACCEL_DLPCFG_6 = 0x30
REG_VAL_BIT_ACCEL_FS_2g = 0x00
REG_VAL_BIT_ACCEL_FS_4g = 0x02
REG_VAL_BIT_ACCEL_FS_8g = 0x04
REG_VAL_BIT_ACCEL_FS_16g = 0x06
REG_VAL_BIT_ACCEL_DLPF = 0x01
REG_ADD_I2C_SLV0_ADDR = 0x03
REG_ADD_I2C_SLV0_REG = 0x04
REG_ADD_I2C_SLV0_CTRL = 0x05
REG_VAL_BIT_SLV0_EN = 0x80
REG_VAL_BIT_MASK_LEN = 0x07
REG_ADD_I2C_SLV0_DO = 0x06
REG_ADD_I2C_SLV1_ADDR = 0x07
REG_ADD_I2C_SLV1_REG = 0x08
REG_ADD_I2C_SLV1_CTRL = 0x09
REG_ADD_I2C_SLV1_DO = 0x0A

# ICM20948
class ICM(object): 
    def __init__(self,address=I2C_ADD_ICM20948):
        self._address = address
        self._bus = smbus.SMBus(1)
        bRet = self.icm20948Check()
        time.sleep(0.5)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        self._write_byte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET)
        time.sleep(0.1)
        self._write_byte(REG_ADD_PWR_MIGMT_1, 0x09)
        self._write_byte(REG_ADD_LP_CONFIG, 0x00)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2)
        self._write_byte(REG_ADD_GYRO_SMPLRT_DIV, 0x00)
        self._write_byte(REG_ADD_GYRO_CONFIG_1, REG_VAL_BIT_GYRO_DLPCFG_2 | REG_VAL_BIT_GYRO_FS_1000DPS | true)
        self._write_byte(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x00)
        self._write_byte(REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_2 | REG_VAL_BIT_ACCEL_FS_8g | true)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        time.sleep(0.1)
        self.icm20948Offset()
        time.sleep(2)
        x = threading.Thread(target=self.read_gamepad)
        x.start()
        self.setup_motors()
    
    def emergency_stop(self):
        emergency_stop[0] = 1
        for i in range(2):
            for i in motor_control:
                i.ChangeDutyCycle(3)

    def read_gamepad(self):
        UDP_IP = "10.42.0.209"
        UDP_PORT = 5005
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        while True:
            data, addr = sock.recvfrom(1024)
            if data.decode('utf-8') == "stop":
                self.emergency_stop()
                break
            value = data.decode('utf-8').split()
            gamepad_output[0] = float(value[0])
            gamepad_output[1] = float(value[1])
            gamepad_output[2] = float(value[2])
            gamepad_output[3] = float(value[3])


    def setup_motors(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        y = 0
        for i in motor_gpio:
            GPIO.setup(i, GPIO.OUT)
            motor_control.append(GPIO.PWM(i, 50))
            motor_control[y].start(0)
            y += 1
        time.sleep(3)
        for i in motor_control:
            i.ChangeDutyCycle(3)
        i = 4
        while i<4.7:
            for y in motor_control:
                y.ChangeDutyCycle(i)
            print(i)
            time.sleep(0.1)
            i += 0.02

    def icm20948_Gyro_Accel_Read(self):
        data = self._read_block(REG_ADD_ACCEL_XOUT_H, 12)
        Accel[0] = data[0] << 8 | data[1]
        Accel[1] = data[2] << 8 | data[3]
        Accel[2] = data[4] << 8 | data[5]
        Gyro[0] = data[6] << 8 | data[7]
        Gyro[1] = data[8] << 8 | data[9]
        Gyro[2] = data[10] << 8 | data[11]
        if Accel[0] >= 32767:
            Accel[0] = Accel[0] - 65535
        elif Accel[0] <= -32767:
            Accel[0] = Accel[0] + 65535
        if Accel[1] >= 32767:
            Accel[1] = Accel[1] - 65535
        elif Accel[1] <= -32767:
            Accel[1] = Accel[1] + 65535
        if Accel[2] >= 32767:
            Accel[2] = Accel[2] - 65535
        elif Accel[2] <= -32767:
            Accel[2] = Accel[2] + 65535
        if Gyro[0] >= 32767:
            Gyro[0] = Gyro[0] - 65535
        elif Gyro[0] <= -32767:
            Gyro[0] = Gyro[0] + 65535
        if Gyro[1] >= 32767:
            Gyro[1] = Gyro[1] - 65535
        elif Gyro[1] <= -32767:
            Gyro[1] = Gyro[1] + 65535
        if Gyro[2] >= 32767:
            Gyro[2] = Gyro[2] - 65535
        elif Gyro[2] <= -32767:
            Gyro[2] = Gyro[2] + 65535
        AccelWindowX.append(Accel[0] / AccelSensitivity - AccelOffset[0])
        AccelWindowY.append(Accel[1] / AccelSensitivity - AccelOffset[1])
        AccelWindowZ.append(Accel[2] / AccelSensitivity - AccelOffset[2])
        Gyro[0] = Gyro[0] / GyroSensitivity - GyroOffset[0]
        Gyro[1] = Gyro[1] / GyroSensitivity - GyroOffset[1]
        Gyro[2] = Gyro[2] / GyroSensitivity - GyroOffset[2]
        if len(AccelWindowX) > 11:
            AccelWindowX.pop(0)
            AccelWindowY.pop(0)
            AccelWindowZ.pop(0)
        Accel[0] = statistics.median(AccelWindowX)
        Accel[1] = statistics.median(AccelWindowY)
        Accel[2] = statistics.median(AccelWindowZ)

    
    def icm20948ReadSecondary(self, u8I2CAddr, u8RegAddr, u8Len):
        u8Temp = 0
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)
        self._write_byte(REG_ADD_I2C_SLV0_ADDR, u8I2CAddr)
        self._write_byte(REG_ADD_I2C_SLV0_REG, u8RegAddr)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8Len)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        u8Temp = self._read_byte(REG_ADD_USER_CTRL)
        u8Temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8Temp)
        time.sleep(0.01)
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8Temp)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)
        u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8Temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, u8Temp)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)

    
    def icm20948WriteSecondary(self, u8I2CAddr, u8RegAddr, u8data):
        u8Temp = 0
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)
        self._write_byte(REG_ADD_I2C_SLV1_ADDR, u8I2CAddr)
        self._write_byte(REG_ADD_I2C_SLV1_REG, u8RegAddr)
        self._write_byte(REG_ADD_I2C_SLV1_DO, u8data)
        self._write_byte(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)
        u8Temp = self._read_byte(REG_ADD_USER_CTRL)
        u8Temp |= REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8Temp)
        time.sleep(0.01)
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
        self._write_byte(REG_ADD_USER_CTRL, u8Temp)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3)
        u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
        u8Temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN)
        self._write_byte(REG_ADD_I2C_SLV0_CTRL, u8Temp)
        self._write_byte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0)

    
    def icm20948Offset(self):
        samples = 1000
        Gx = 0
        Gy = 0
        Gz = 0
        Ax = 0
        Ay = 0
        Az = 0
        for i in range(samples):
            self.icm20948_Gyro_Accel_Read()
            Gx += Gyro[0]
            Gy += Gyro[1]
            Gz += Gyro[2]
            Ax += Accel[0]
            Ay += Accel[1]
            Az += Accel[2]
        GyroOffset[0] = Gx / samples
        GyroOffset[1] = Gy / samples
        GyroOffset[2] = Gz / samples
        AccelOffset[0] = Ax / samples
        AccelOffset[1] = Ay / samples
        AccelOffset[2] = Az / samples - 1
        print("Calibrated")

    
    def _read_byte(self, cmd):
        return self._bus.read_byte_data(self._address,cmd)

    
    def _read_block(self, reg, length = 1):
        return self._bus.read_i2c_block_data(self._address, reg, length)

    
    def _read_u16(self, cmd):
        LSB = self._bus.read_byte_data(self._address, cmd)
        MSB = self._bus.read_byte_data(self._address, cmd + 1)
        return (MSB << 8) + LSB

    
    def _write_byte(self, cmd, val):
        self._bus.write_byte_data(self._address,cmd,val)
        time.sleep(0.0001)

    
    def icm20948Check(self):
        bRet = false
        if REG_VAL_WIA == self._read_byte(REG_ADD_WIA):
            bRet = true
        return bRet

    
    def Get_Gyro_Angles(self, delta_time):
        GyroAngles[0] += Gyro[0] * delta_time
        GyroAngles[1] += Gyro[1] * delta_time
        GyroAngles[2] += Gyro[2] * delta_time

    
    def Get_Accel_Angles(self):
        x = Accel[0]
        y = Accel[1]
        z = Accel[2]
        AccelAngles[0] = math.atan2(y, z) * RAD_TO_DEG
        AccelAngles[1] = math.atan2(-1 * x, math.sqrt(y * y + z * z)) * RAD_TO_DEG

    
    def Get_Angles(self, delta_time):
        GYRO_PART = 0.995
        ACCEL_PART = 0.005
        Angles[0] = GYRO_PART * (Angles[0] + Gyro[0] * delta_time) + ACCEL_PART * AccelAngles[0]
        Angles[1] = GYRO_PART * (Angles[1] + Gyro[1] * delta_time) + ACCEL_PART * AccelAngles[1]
        Angles[2] = GyroAngles[2]

    
    def Calculate(self, delta_time):
        self.icm20948_Gyro_Accel_Read()
        self.Get_Gyro_Angles(delta_time)
        self.Get_Accel_Angles()
        self.Get_Angles(delta_time)


    def Angle_PID(self, delta_time, axis, desired_angle, KP, KI, KD):
        error = desired_angle - Angles[axis]
        error = error * 4
        return self.Rate_PID(delta_time, axis, error, KP, KI, KD)


    def Rate_PID(self, delta_time, axis, desired_pitch, KP, KI, KD):
        error = desired_pitch - Gyro[axis]
        
        proportional = KP * error
        rate_pid_integral[axis] += KI * error * delta_time
        derivative = KD * (error - GyroPrev[axis] - GamepadPrev[axis])

        GamepadPrev[axis] = desired_pitch
        GyroPrev[axis] = Gyro[axis]

        return proportional + rate_pid_integral[axis] + derivative


if __name__ == '__main__':
    icm = ICM()
    iteration = 0
    delta_t = 0
    last_update = time.perf_counter()
    while True: 
        # Stop if emergency stop
        if emergency_stop[0] == 1:
            print("emergency stop")
            break

        # Calculate delta time and set last update time
        delta_t = time.perf_counter() - last_update  
        last_update = time.perf_counter()
        
        # Calculate read data and angles
        icm.Calculate(delta_t)
       
        # Calculate required thrust, roll, pitch and yaw
        Thrust = gamepad_output[0]
        Roll = icm.Angle_PID(delta_t, 0, gamepad_output[1], KP_roll, KI_roll, KD_roll)
        Pitch = icm.Angle_PID(delta_t, 1, gamepad_output[2], KP_pitch, KI_pitch, KD_pitch)
        Yaw = icm.Rate_PID(delta_t, 0, gamepad_output[3], KP_yaw, KI_yaw, KD_yaw)

        # Calculate thrust for each motor
        motor[0] = Thrust + Roll - Pitch - Yaw
        motor[1] = Thrust - Roll - Pitch + Yaw
        motor[2] = Thrust - Roll + Pitch - Yaw
        motor[3] = Thrust + Roll + Pitch + Yaw

        # Avoid overflow
        for i in range(len(motor)):
            if motor[i] > 10:
                motor[i] = 10
            elif motor[i] < 5:
                motor[i] = 5

        # Set thrust for each motor
        for i in range(len(motor_control)):
            motor_control[i].ChangeDutyCycle(motor[i])

        # Print some data every couple of times
        if iteration%200 == 0:
            print(motor[0], motor[1])
            print(motor[3], motor[2])
            print(Thrust, Roll, Pitch, Yaw)
            print("\n")
        iteration += 1
