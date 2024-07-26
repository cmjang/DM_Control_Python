import time
import numpy as np
from enum import IntEnum
import struct


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM6006 = 3
    DM8006 = 4
    DM8009 = 5


class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4


# 电机参数限制
class Limit_Motor:
    def __init__(self, P_MAX: float, V_MAX: float, T_MAX: float):
        self.Q_MAX = P_MAX
        self.DQ_MAX = V_MAX
        self.TAU_MAX = T_MAX


DM_4310_Limit = Limit_Motor(12.5, 30, 10)
DM_4310_48V_Limit = Limit_Motor(12.5, 50, 10)
DM_4340_Limit = Limit_Motor(12.5, 10, 28)
DM_6006_Limit = Limit_Motor(12.5, 45, 20)
DM_8006_Limit = Limit_Motor(12.5, 45, 40)
DM_8009_Limit = Limit_Motor(12.5, 45, 54)


class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        """
        define Motor object 定义电机对象
        :param MotorType: Motor type 电机类型
        :param SlaveID: CANID 电机ID
        :param MasterID: MasterID 主机ID 建议不要设为0
        """
        self.Pd = float(0)
        self.Vd = float(0)
        self.cmd_q = float(0)
        self.cmd_dq = float(0)
        self.cmd_tau = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.cmd_kp = float(0)
        self.cmd_kd = float(0)
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType

    def recv_data(self, q: float, dq: float, tau: float):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

    def save_cmd(self, cmd_kp: float, cmd_kd: float, q: float, dq: float, tau: float):
        self.cmd_q = q
        self.cmd_dq = dq
        self.cmd_tau = tau
        self.cmd_kp = cmd_kp
        self.cmd_kd = cmd_kd

    def getPosition(self):
        """
        get the position of the motor 获取电机位置
        :return: the position of the motor 电机位置
        """
        return self.state_q

    def getVelocity(self):
        """
        get the velocity of the motor 获取电机速度
        :return: the velocity of the motor 电机速度
        """
        return self.state_dq

    def getTorque(self):
        """
        get the torque of the motor 获取电机力矩
        :return: the torque of the motor 电机力矩
        """
        return self.state_tau


# -------------------------------------------------
# Extract packets from the serial data
def extract_packets(data):
    packets = []
    header = 0xAA
    tail = 0x55
    i = 0
    while i < len(data) - 1:
        # Find the start of a packet
        if data[i] == header:
            start_index = i
            # Look for the end of the packet
            i += 1
            while i < len(data) and data[i] != tail:
                i += 1
            # If a tail is found, extract the packet
            if i < len(data) and data[i] == tail:
                end_index = i
                packets.append(data[start_index:end_index + 1])
        i += 1
    return packets


Motor_Param_limits = [DM_4310_Limit, DM_4310_48V_Limit, DM_4340_Limit, DM_6006_Limit, DM_8006_Limit, DM_8009_Limit]


class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)

    def __init__(self, serial_device):
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.serial_ = serial_device
        self.motors_map = dict()
        if self.serial_.is_open:  # open the serial port
            print("Serial port is open")
            serial_device.close()
        self.serial_.open()

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        MIT Control Mode Function 达妙电机MIT控制模式函数
        :param DM_Motor: Motor object 电机对象
        :param kp: kp
        :param kd:  kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :return: None
        """
        if DM_Motor.SlaveID not in self.motors_map:
            print("Motor ID not found")
            return
        DM_Motor.save_cmd(kp, kd, q, dq, tau)
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        # print(type(DM_Motor.MotorType))
        MotorType = DM_Motor.MotorType
        q_uint = float_to_uint(q, -Motor_Param_limits[MotorType].Q_MAX, Motor_Param_limits[MotorType].Q_MAX,
                               16)
        dq_uint = float_to_uint(dq, -Motor_Param_limits[MotorType].DQ_MAX, Motor_Param_limits[MotorType].DQ_MAX,
                                12)
        tau_uint = float_to_uint(tau, -Motor_Param_limits[MotorType].TAU_MAX, Motor_Param_limits[MotorType].TAU_MAX,
                                 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.send_data_frame[13] = DM_Motor.SlaveID
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        MIT Control Mode Function with delay 达妙电机MIT控制模式函数带延迟
        :param DM_Motor: Motor object 电机对象
        :param kp: kp
        :param kd: kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :param delay: delay time 延迟时间 单位秒
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        time.sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        control the motor in position and velocity control mode 电机位置速度控制模式
        :param Motor: Motor object 电机对象
        :param P_desired: desired position 期望位置
        :param V_desired: desired velocity 期望速度
        :return: None
        """
        if Motor.SlaveID not in self.motors_map:
            print("Motor ID not found")
            return
        self.send_data_frame[13] = Motor.SlaveID
        self.send_data_frame[14] = 0x01  # vel pos control need 0x100+canid
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port

    def control_Vel(self, Motor, Vel_desired):
        """
        control the motor in velocity control mode 电机速度控制模式
        :param Motor: Motor object 电机对象
        :param Vel_desired: desired velocity 期望速度
        """
        if Motor.SlaveID not in self.motors_map:
            print("Motor ID not found")
            return
        self.send_data_frame[13] = Motor.SlaveID
        self.send_data_frame[14] = 0x02  # vel control need 0x200+canid
        data_buf = np.array([0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0] = Vel_desired_uint8s[0]
        data_buf[1] = Vel_desired_uint8s[1]
        data_buf[2] = Vel_desired_uint8s[2]
        data_buf[3] = Vel_desired_uint8s[3]
        self.send_data_frame[21:25] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port

    def control_pos_force(self,Motor,Pos_des:float,Vel_des,i_des):
        """
        control the motor in EMIT control mode 电机力位混合模式
        :param Pos_des: desired position rad  期望位置 单位为rad
        :param Vel_des: desired velocity rad/s  期望速度 为放大100倍
        :param i_des: desired current rang 0-10000 期望电流标幺值放大10000倍
        电流标幺值：实际电流值除以最大电流值，最大电流见上电打印
        """
        if Motor.SlaveID not in self.motors_map:
            print("Motor ID not found")
            return
        self.send_data_frame[13] = Motor.SlaveID
        self.send_data_frame[14] = 0x03  # vel control need 0x200+canid
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s

        Vel_uint= np.uint16(Vel_des)
        ides_uint= np.uint16(i_des)
        data_buf[4]=Vel_uint & 0xff
        data_buf[5]=Vel_uint >> 8
        data_buf[6]=ides_uint&0xff
        data_buf[7]=ides_uint>>8
        print(data_buf)
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port


    def enable(self, Motor):
        """
        enable motor 使能电机
        最好在上电后几秒后再使能电机
        :param Motor: Motor object 电机对象
        """
        self.control_cmd(Motor, np.uint8(0xFC))

    def disable(self, Motor):
        """
        disable motor 失能电机
        :param Motor: Motor object 电机对象
        """
        self.control_cmd(Motor, np.uint8(0xFD))

    def zero_position(self, Motor):
        """
        set the zero position of the motor 设置电机0位
        :param Motor: Motor object 电机对象
        """
        self.control_cmd(Motor, np.uint8(0xFE))

    def recv(self):
        data_recv = self.serial_.read_all()
        packets = extract_packets(data_recv)
        for packet in packets:
            if len(packet) == 16:
                data = packet[7:15]
                CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
                CMD = packet[1]
                self.process_packet(data, CANID, CMD)

    def process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID in self.motors_map:
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                MotorType_recv = self.motors_map[CANID].MotorType
                recv_q = uint_to_float(q_uint, -Motor_Param_limits[MotorType_recv].Q_MAX,
                                       Motor_Param_limits[MotorType_recv].Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, -Motor_Param_limits[MotorType_recv].DQ_MAX,
                                        Motor_Param_limits[MotorType_recv].DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -Motor_Param_limits[MotorType_recv].TAU_MAX,
                                         Motor_Param_limits[MotorType_recv].TAU_MAX, 12)
                self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)

    def addMotor(self, Motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param Motor: Motor object 电机对象
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor

    def control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.send_data_frame[13] = np.uint8(Motor.SlaveID)
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port

    def read_motor_param(self, Motor, RID):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.send_data_frame[13] = 0xFF
        self.send_data_frame[14] = 0x07
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))

    #   self.recv()  # receive the data from serial port

    def write_motor_param(self, Motor, RID, data):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[4:8] = data
        self.send_data_frame[13] = 0xFF
        self.send_data_frame[14] = 0x07
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))

    def switchControlMode(self, Motor, ControlMode):
        """
        switch the control mode of the motor 切换电机控制模式
        :param Motor: Motor object 电机对象
        :param ControlMode: Control_Type 电机控制模式 example:MIT:Control_Type.MIT MIT模式
        """
        write_data = np.array([np.uint8(ControlMode), 0x00, 0x00, 0x00], np.uint8)
        self.write_motor_param(Motor, 10, write_data)

    def save_motor_param(self, Motor):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.send_data_frame[13] = 0xFF
        self.send_data_frame[14] = 0x07
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()


def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        x = min
    elif x > max:
        x = max


def float_to_uint(x: float, x_min: float, x_max: float, bits):
    LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value):
    # Pack the float into 4 bytes
    packed = struct.pack('f', value)
    # Unpack the bytes into four uint8 values
    return struct.unpack('4B', packed)
