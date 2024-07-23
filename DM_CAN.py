import time
import numpy as np
from enum import IntEnum


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM6006 = 3
    DM8006 = 4
    DM8009 = 5


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
        self.serial_ = serial_device
        self.motors_map = dict()
        if self.serial_.is_open:  # open the serial port
            print("Serial port is open")
            serial_device.close()
        self.serial_.open()

    # -------------------------------------------------
    # Control the motor
    # @param DM_Motor: the motor to be controlled
    # @param kp: the proportional gain 期望的kp
    # @param kd: the derivative gain  期望的kd
    # @param q: the desired position  期望位置
    # @param dq: the desired velocity 期望速度
    # @param tau: the desired torque  期望力矩
    def control(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        if DM_Motor.SlaveID not in self.motors_map:
            print("Motor ID not found")
            return
        DM_Motor.save_cmd(kp, kd, q, dq, tau)
        kp_uint = float_to_uint(kp, 0, 500, np.uint8(12))
        kd_uint = float_to_uint(kd, 0, 5, np.uint8(12))
        # print(type(DM_Motor.MotorType))
        MotorType = DM_Motor.MotorType
        q_uint = float_to_uint(q, -Motor_Param_limits[MotorType].Q_MAX, Motor_Param_limits[MotorType].Q_MAX,
                               np.uint8(16))
        dq_uint = float_to_uint(dq, -Motor_Param_limits[MotorType].DQ_MAX, Motor_Param_limits[MotorType].DQ_MAX,
                                np.uint8(12))
        tau_uint = float_to_uint(tau, -Motor_Param_limits[MotorType].TAU_MAX, Motor_Param_limits[MotorType].TAU_MAX,
                                 np.uint8(12))
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
            self.control(DM_Motor, kp, kd, q, dq, tau)
            time.sleep(delay)

    # enable motor  使能电机
    def enable(self, Motor):
        self.control_cmd(Motor, np.uint8(0xFC))

    # disable motor  关闭电机
    def disable(self, Motor):
        self.control_cmd(Motor, np.uint8(0xFD))

    # set zero position  设置零点
    def zero_position(self, Motor):
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
                # print("CANID: ", hex(CANID), "CMD: ", hex(CMD))

    def process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID in self.motors_map:
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                MotorType_recv = self.motors_map[CANID].MotorType
                recv_q = uint_to_float(q_uint, -Motor_Param_limits[MotorType_recv].Q_MAX,
                                       Motor_Param_limits[MotorType_recv].Q_MAX, np.uint8(16))
                recv_dq = uint_to_float(dq_uint, -Motor_Param_limits[MotorType_recv].DQ_MAX,
                                        Motor_Param_limits[MotorType_recv].DQ_MAX, np.uint8(12))
                recv_tau = uint_to_float(tau_uint, -Motor_Param_limits[MotorType_recv].TAU_MAX,
                                         Motor_Param_limits[MotorType_recv].TAU_MAX, np.uint8(12))
                self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)

    # add motor to the motor control 添加电机
    def addMotor(self, Motor):
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor

    def control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.send_data_frame[13] = np.uint8(Motor.SlaveID)
        self.send_data_frame[21:29] = data_buf
        self.serial_.write(bytes(self.send_data_frame.T))
        self.recv()  # receive the data from serial port


def float_to_uint(x: float, x_min: float, x_max: float, bits: np.uint8):
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, xmin: float, xmax: float, bits: np.uint8):
    span = xmax - xmin
    data_norm = float(x) / ((1 << bits) - 1)
    return np.float32(data_norm * span + xmin)
