import time
import numpy as np
from enum import IntEnum
import struct


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
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isenable = False
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}

    def recv_data(self, q: float, dq: float, tau: float):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

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

    def getParam(self, RID):
        """
        get the parameter of the motor 获取电机内部的参数，需要提前读取
        :param RID: DM_variable 电机参数
        :return: the parameter of the motor 电机参数
        """
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None


class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
    #                4310           4310_48        4340           4340_48
    Limit_Param = [[12.5, 30, 10], [12.5, 50, 10], [12.5, 8, 28], [12.5, 10, 28],
                   # 6006           8006           8009
                   [12.5, 45, 20], [12.5, 45, 40], 12.5, 45, 54]

    def __init__(self, serial_device):
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.serial_ = serial_device
        self.motors_map = dict()
        self.data_save = bytes()  # save data
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
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
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
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        # time.sleep(0.001)
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
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
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
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def enable(self, Motor):
        """
        enable motor 使能电机
        最好在上电后几秒后再使能电机
        :param Motor: Motor object 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        time.sleep(0.1)

    def disable(self, Motor):
        """
        disable motor 失能电机
        :param Motor: Motor object 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFD))

    def zero_position(self, Motor):
        """
        set the zero position of the motor 设置电机0位
        :param Motor: Motor object 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFE))

    def recv(self):
        # 把上次没有解析完的剩下的也放进来
        data_recv = self.data_save + self.serial_.read_all()
        packets = self.extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        data_recv = self.serial_.read_all()
        packets = self.extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID in self.motors_map:
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                MotorType_recv = self.motors_map[CANID].MotorType
                Q_MAX = self.Limit_Param[MotorType_recv][0]
                DQ_MAX = self.Limit_Param[MotorType_recv][1]
                TAU_MAX = self.Limit_Param[MotorType_recv][2]
                recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)

    def __process_set_param_packet(self, data, CANID, CMD):
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            masterid_temp = ((data[1] << 8) | data[0])
            if CANID in self.motors_map or masterid_temp in self.motors_map:
                RID = data[3]
                # 读取参数得到的数据
                if is_in_ranges(RID):
                    #uint32类型
                    num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                    self.motors_map[CANID].temp_param_dict[RID] = num
                    return True
                else:
                    #float类型
                    num = uint8s_to_float(data[4], data[5], data[6], data[7])
                    self.motors_map[CANID].temp_param_dict[RID] = num
                    return True
            else:
                return False
        else:
            return False

    def addMotor(self, Motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param Motor: Motor object 电机对象
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)
        self.recv()  # receive the data from serial port

    def __send_data(self, motorid, data):
        """
        send data to the motor 发送数据到电机
        :param motorid:
        :param data:
        :return:
        """
        self.send_data_frame[13] = motorid & 0xff
        self.send_data_frame[14] = motorid >> 8
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def read_RID_param(self, Motor, RID):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        time.sleep(0.1)

    def __write_motor_param(self, Motor, RID, data):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        # data_buf[4:8] = data
        if not is_in_ranges(RID):
            # data is float
            data_buf[4:8] = float_to_uint8s(data)
        else:
            # data is int
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        switch the control mode of the motor 切换电机控制模式
        :param Motor: Motor object 电机对象
        :param ControlMode: Control_Type 电机控制模式 example:MIT:Control_Type.MIT MIT模式
        """
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        time.sleep(0.01)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map:
            if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == ControlMode:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    def save_motor_param(self, Motor):
        data_buf = np.array([np.uint8(Motor.SlaveID), 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        time.sleep(0.1)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        change the PMAX VMAX TMAX of the motor 改变电机的PMAX VMAX TMAX
        :param Motor_Type:
        :param PMAX: 电机的PMAX
        :param VMAX: 电机的VMAX
        :param TMAX: 电机的TMAX
        :return:
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def change_PMAX(self, Motor_Type, PMAX):
        """
        change the PMAX of the motor 改变电机控制的PMAX
        :param Motor_Type:
        :param PMAX: 电机的PMAX
        :return:
        """
        self.Limit_Param[Motor_Type][0] = PMAX

    def change_motor_param(self, Motor, RID, data):
        """
        change the RID of the motor 改变电机的参数
        :param Motor: Motor object 电机对象
        :param RID: DM_variable 电机参数
        :param data: 电机参数的值
        :return: True or False ,True means success, False means fail
        """
        self.__write_motor_param(Motor, RID, data)
        time.sleep(0.1)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map:
            if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == data:
                return True
            else:
                return False
        else:
            return False

    def read_motor_param(self, Motor, RID):
        """
        read only the RID of the motor 读取电机的内部信息例如 版本号等
        :param Motor: Motor object 电机对象
        :param RID: DM_variable 电机参数
        :return: 电机参数的值
        """
        self.read_RID_param(Motor, RID)
        time.sleep(0.01)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map:
            if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
            else:
                return None
        else:
            return None
    # -------------------------------------------------
    # Extract packets from the serial data
    def extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        # print("data_save:", self.data_save)
        return frames


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


def data_to_uint8s(value):
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = struct.pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")

    # Unpack the bytes into four uint8 values
    return struct.unpack('4B', packed)


def is_in_ranges(number):
    """
    check if the number is in the range of uint32
    :param number:
    :return:
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = struct.pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return struct.unpack('<I', packed)[0]


def uint8s_to_float(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single float value in little-endian order
    packed = struct.pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return struct.unpack('<f', packed)[0]


def print_hex(data):
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6


class DM_variable(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81


class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4
