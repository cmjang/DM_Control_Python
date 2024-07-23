import math
from DM_CAN import *
import serial
import time

Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
Motor2=Motor(DM_Motor_Type.DM4310,0x02,0x12)
Motor3=Motor(DM_Motor_Type.DM4310,0x03,0x13)
serial_device = serial.Serial('COM8', 921600, timeout=0.5)
MotorControl1=MotorControl(serial_device)
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)

while True:
    q=math.sin(time.time())
    MotorControl1.control(Motor1, 50, 0.3, q, 0, 0)
    MotorControl1.control(Motor2, 50, 0.3, q, 0, 0)
    MotorControl1.control(Motor3, 50, 0.3, q, 0, 0)

#语句结束关闭串口
serial_device.close()