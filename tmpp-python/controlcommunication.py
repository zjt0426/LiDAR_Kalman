# -*- coding: utf-8 -*-

import numpy as np
import math
import time
import socket


def euler_to_rotation_matrix(euler_angles):
    phi, theta, psi = euler_angles
    R_z = np.array([[math.cos(psi), -math.sin(psi), 0],
                    [math.sin(psi), math.cos(psi), 0],
                    [0, 0, 1]])
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta), -math.sin(theta)],
                    [0, math.sin(theta), math.cos(theta)]])
    R_y = np.array([[math.cos(phi), 0, math.sin(phi)],
                    [0, 1, 0],
                    [-math.sin(phi), 0, math.cos(phi)]])
    R = np.dot(R_z, np.dot(R_x, R_y))
    return R

# 定义从旋转矩阵提取 z-x-y 欧拉角的函数
def rotation_matrix_to_euler(R):
    # 使用 zxy 顺序，首先计算 yaw（psi），然后 pitch（theta），最后 roll（phi）
    theta = -math.asin(R[2, 0])
    if math.cos(theta) != 0:
        phi = math.atan2(R[2, 1] / math.cos(theta), R[2, 2] / math.cos(theta))
        psi = math.atan2(R[1, 0] / math.cos(theta), R[0, 0] / math.cos(theta))
    else:
        phi = 0
        psi = math.atan2(-R[0, 1], R[1, 1])
    return [phi, theta, psi]
class Tunneling:
    def __init__(self, x, y, z, CenterToB1, NeToRoad_angle, NeToIns_angle, Hexahedron, RoadParameter):
        # 载体在巷道坐标系中坐标
        self.x = x
        self.y = y
        self.z = z
        # 机心相对于载体坐标——载体坐标系
        self.CentertoB1_B = CenterToB1
        # 东北天到巷道欧拉角（惯导给出--东北天到巷道注意方向）
        self.netoroad_angle = NeToRoad_angle
        # 东北天到惯导欧拉角（惯导给出--东北天到惯导注意方向，要取逆）
        self.netoins_angle = NeToIns_angle    # 312  偏航，俯仰，横滚
        # 外接六面体8个顶点坐标(在机心原点处）（排列顺序为前铲板左侧，前铲板右侧，车尾左侧，车尾右侧）数组形式并从高到底排序
        self.hexahedron = Hexahedron
        self.roadwidth = RoadParameter[0]/2.0
        self.roadheight = RoadParameter[1]
        self.NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        self.NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)

    def euler_matrix(self, angle, flag):
        # 这里的angle表示欧拉角的三个角度翻滚角、俯仰角以及偏航角。并且考虑正向YXZ输入顺序。
        # 规定惯导和东北天之间的夹角为：惯导输出的角度为东北天到惯导的角度。
        # flag :True--东北天到惯导或者巷道(zxy)    False--惯导到东北天角度
        a, b, c = angle[0], angle[1], angle[2]
        # Y
        a = math.radians(a)
        b = math.radians(b)
        c = math.radians(c)
        A = np.array([[np.cos(a), 0, -np.sin(a)],
                      [0, 1, 0],
                      [np.sin(a), 0, np.cos(a)]])
        # X
        B = np.array([[1, 0, 0],
                      [0, np.cos(b), np.sin(b)],
                      [0, -np.sin(b), np.cos(b)]])
        # Z
        C = np.array([[np.cos(c), np.sin(c), 0],
                      [-np.sin(c), np.cos(c), 0],
                      [0, 0, 1]])
        C_ = C.T
        A_ = A.T
        B_ = B.T
        return np.dot(np.dot(A, B), C) if flag else np.dot(np.dot(A_, B_), C_)
    def ForwardDistance(self, MWR_dist, HeadToMWR):
        # MWR为距离前向距离
        # HeadToMWR表示炮头到该测距毫米波雷达距离
        for_dist = MWR_dist - HeadToMWR
        return for_dist
    def RearDistance(self, reardist):
        # tmp = reardist
        # 经验值给出
        return reardist

    def ShovelPlateLeftToWall(self, x_add):
        shovelLeftToCenterUp = self.hexahedron[0]
        shovelLeftToCenterDown = self.hexahedron[1]
        shovelLeftUpToB_B = shovelLeftToCenterUp + self.CentertoB1_B
        shovelLeftDownToB_B = shovelLeftToCenterDown + self.CentertoB1_B
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        shovelLeftUpToB_N = np.dot(self.NeToIns_mat, shovelLeftUpToB_B)
        shovelLeftUpToB_Road = np.dot(self.NeToRoad_mat, shovelLeftUpToB_N)
        # carx = self.Car_x_Coordinate()
        shovelLeftDownToB_N = np.dot(self.NeToIns_mat, shovelLeftDownToB_B)
        shovelLeftDownToB_Road = np.dot(self.NeToRoad_mat, shovelLeftDownToB_N)
        # 近
        # shovelLeft = min(shovelLeftUpToB_Road[0] + carx + self.roadwidth, shovelLeftDownToB_Road[0] + carx + self.roadwidth)

        shovelLeft = min(shovelLeftUpToB_Road[0] + self.x + self.roadwidth + x_add,
                         shovelLeftDownToB_Road[0] + self.x + self.roadwidth + x_add)
        # shovelLeft = min(shovelLeftUpToB_Road[0] + self.x + self.roadwidth, shovelLeftDownToB_Road[0] + self.x + self.roadwidth)
        return shovelLeft

    def ShovelPlateRightToWall(self, x_add):
        shovelRightToCenterUp = self.hexahedron[2]
        shovelRightToCenterDown = self.hexahedron[3]
        shovelRightUpToB_B = shovelRightToCenterUp + self.CentertoB1_B
        shovelRightDownToB_B = shovelRightToCenterDown + self.CentertoB1_B
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        shovelRightUpToB_N = np.dot(self.NeToIns_mat, shovelRightUpToB_B)
        shovelRightUpToB_Road = np.dot(self.NeToRoad_mat, shovelRightUpToB_N)
        # carx = self.Car_x_Coordinate()
        shovelRightDownToB_N = np.dot(self.NeToIns_mat, shovelRightDownToB_B)
        shovelRightDownToB_Road = np.dot(self.NeToRoad_mat, shovelRightDownToB_N)

        # shovelRight = min(self.roadwidth - (shovelRightUpToB_Road[0] + carx), self.roadwidth - (shovelRightDownToB_Road[0] + carx))
        shovelRight = min(self.roadwidth - (shovelRightUpToB_Road[0] + self.x + x_add),
                          self.roadwidth - (shovelRightDownToB_Road[0] + self.x + x_add))
        # shovelRight = min(self.roadwidth - (shovelRightUpToB_Road[0] + self.x), self.roadwidth - (shovelRightDownToB_Road[0] + self.x))

        return shovelRight

    def RearLeftToWall(self, x_add):
        rearLeftToCenterUp = self.hexahedron[4]
        rearLeftToCenterDown = self.hexahedron[5]
        rearLeftUpToB_B = rearLeftToCenterUp + self.CentertoB1_B
        rearLeftDownToB_B = rearLeftToCenterDown + self.CentertoB1_B
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        rearLeftUpToB_N = np.dot(self.NeToIns_mat, rearLeftUpToB_B)
        rearLeftUpToB_Road = np.dot(self.NeToRoad_mat, rearLeftUpToB_N)
        # carx = self.Car_x_Coordinate()
        rearLeftDownToB_N = np.dot(self.NeToIns_mat, rearLeftDownToB_B)
        rearLeftDownToB_Road = np.dot(self.NeToRoad_mat, rearLeftDownToB_N)
        # rearLeft = min(rearLeftUpToB_Road[0] + carx + self.roadwidth, rearLeftDownToB_Road[0] + carx + self.roadwidth)

        rearLeft = min(rearLeftUpToB_Road[0] + self.x + self.roadwidth + x_add,
                       rearLeftDownToB_Road[0] + self.x + self.roadwidth + x_add)
        # rearLeft = min(rearLeftUpToB_Road[0] + self.x + self.roadwidth, rearLeftDownToB_Road[0] + self.x + self.roadwidth)
        return rearLeft

    def RearRightToWall(self, x_add):
        rearRightToCenterUp = self.hexahedron[6]
        rearRightToCenterDown = self.hexahedron[7]
        rearRightUpToB_B = rearRightToCenterUp + self.CentertoB1_B
        rearRightDownToB_B = rearRightToCenterDown + self.CentertoB1_B
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        rearRightUpToB_N = np.dot(self.NeToIns_mat, rearRightUpToB_B)
        rearRightUpToB_Road = np.dot(self.NeToRoad_mat, rearRightUpToB_N)
        rearRightDownToB_N = np.dot(self.NeToIns_mat, rearRightDownToB_B)
        # carx = self.Car_x_Coordinate()
        rearRightDownToB_Road = np.dot(self.NeToRoad_mat, rearRightDownToB_N)
        # rearRight = min(self.roadwidth - (rearRightUpToB_Road[0] + carx), self.roadwidth - (rearRightDownToB_Road[0] + carx))

        rearRight = min(self.roadwidth - (rearRightUpToB_Road[0] + self.x + x_add),
                        self.roadwidth - (rearRightDownToB_Road[0] + self.x + x_add))
        # rearRight = min(self.roadwidth - (rearRightUpToB_Road[0] + self.x), self.roadwidth - (rearRightDownToB_Road[0] + self.x))
        return rearRight

    # 机心距离左侧巷道壁距离
    def CenterToLeftWall(self, x_add):
        center_Road = self.Car_x_Coordinate() + x_add
        # center_Road = self.Car_x_Coordinate()

        centerToleftwall = center_Road + self.roadwidth
        return centerToleftwall

    # 机心距离右侧巷道壁距离
    def CenterToRightWall(self, x_add):
        center_Road = self.Car_x_Coordinate() + x_add
        # center_Road = self.Car_x_Coordinate()
        centerTorightwall = self.roadwidth - center_Road
        return centerTorightwall

    # 机心距离中轴线距离
    def CenterToCenterline(self, x_add):
        center_Road = self.Car_x_Coordinate() + x_add
        # center_Road = self.Car_x_Coordinate()
        return center_Road

    # 巷道坐标系xoy平面在哪？重新计算
    def CarbodyToRoof(self, Lidar1_z, z_add):
        # 首先计算车心在巷道坐标下的z坐标，然后换算到顶板处。和车心z坐标一致。
        lidarToroof = self.Car_z_Coordinate() - z_add
        # lidarToroof = self.roadheight - Lidar1_z
        return lidarToroof

    # 计算机心x坐标
    def Car_x_Coordinate(self):
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        self.CentertoB1_N = np.dot(self.NeToIns_mat, self.CentertoB1_B)
        self.CentertoB1_Road = np.dot(self.NeToRoad_mat, self.CentertoB1_N)
        carx = self.CentertoB1_Road[0] + self.x
        return carx

    # 计算机心y坐标
    def Car_z_Coordinate(self):
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        self.CentertoB1_N = np.dot(self.NeToIns_mat, self.CentertoB1_B)
        self.CentertoB1_Road = np.dot(self.NeToRoad_mat, self.CentertoB1_N)
        carz = self.CentertoB1_Road[2] + self.z
        return carz

    def Car_y_Coordinate(self):
        # NeToIns_mat = self.euler_matrix(self.netoins_angle[::-1], False)
        # NeToRoad_mat = self.euler_matrix(self.netoroad_angle[::-1], True)
        self.CentertoB1_N = np.dot(self.NeToIns_mat, self.CentertoB1_B)
        self.CentertoB1_Road = np.dot(self.NeToRoad_mat, self.CentertoB1_N)
        cary = self.CentertoB1_Road[1] + self.y
        return cary
    # 312  偏航，俯仰，横滚  Yaw, Pitch, Roll
    def Yaw_Pitch_Roll(self):
        yaw_deg, pitch_deg, roll_deg = self.netoins_angle[0], self.netoins_angle[1], self.netoins_angle[2]
        return yaw_deg, pitch_deg, roll_deg

def negativeAndPositive_2bytes(n):
    n = int(n * 1000)
    # print(n)
    # 将绝对值转换为二进制
    abs_binary = bin(abs(n))[2:]
    # 补齐至16位
    abs_binary = abs_binary.zfill(16)
    # 反转二进制数的每一位
    inverted_binary = ''.join('0' if b == '1' else '1' for b in abs_binary)
    # 加1操作
    twos_complement = bin(int(inverted_binary, 2) + 1)[2:]
    # 补齐至16位
    twos_complement = twos_complement.zfill(16)
    # 输出高低位的值
    high_byte = twos_complement[:8]
    low_byte = twos_complement[8:16]

    decimal_high = int(high_byte, 2)
    decimal_low = int(low_byte, 2)
    return decimal_high, decimal_low

def negativeAndPositive_2bytes_angle(n):
    n = int(n * 10)
    # print(n)
    # 将绝对值转换为二进制
    abs_binary = bin(abs(n))[2:]
    # 补齐至16位
    abs_binary = abs_binary.zfill(16)
    # 反转二进制数的每一位
    inverted_binary = ''.join('0' if b == '1' else '1' for b in abs_binary)
    # 加1操作
    twos_complement = bin(int(inverted_binary, 2) + 1)[2:]
    # 补齐至16位
    twos_complement = twos_complement.zfill(16)
    # 输出高低位的值
    high_byte = twos_complement[:8]
    low_byte = twos_complement[8:16]

    decimal_high = int(high_byte, 2)
    decimal_low = int(low_byte, 2)
    return decimal_high, decimal_low

def negativeAndPositive_3bytes(n):
    n = int(n * 100)
    # 将绝对值转换为二进制
    abs_binary = bin(abs(n))[2:]
    # 补齐至16位
    abs_binary = abs_binary.zfill(24)
    # 反转二进制数的每一位
    inverted_binary = ''.join('0' if b == '1' else '1' for b in abs_binary)
    # 加1操作
    twos_complement = bin(int(inverted_binary, 2) + 1)[2:]
    # 补齐至16位
    twos_complement = twos_complement.zfill(24)
    # 输出高低位的值
    high_byte = twos_complement[:8]
    mid_byte = twos_complement[8:16]
    low_byte = twos_complement[16:24]

    decimal_high = int(high_byte, 2)
    decimal_mid = int(mid_byte, 2)
    decimal_low = int(low_byte, 2)
    return decimal_low, decimal_mid, decimal_high

def negativeAndPositive_2bytes_no_big(n):
    n = int(n)
    # print(n)
    # 将绝对值转换为二进制
    abs_binary = bin(abs(n))[2:]
    # 补齐至16位
    abs_binary = abs_binary.zfill(16)
    # 反转二进制数的每一位
    inverted_binary = ''.join('0' if b == '1' else '1' for b in abs_binary)
    # 加1操作
    twos_complement = bin(int(inverted_binary, 2) + 1)[2:]
    # 补齐至16位
    twos_complement = twos_complement.zfill(16)
    # 输出高低位的值
    high_byte = twos_complement[:8]
    low_byte = twos_complement[8:16]

    decimal_high = int(high_byte, 2)
    decimal_low = int(low_byte, 2)
    return decimal_low, decimal_high

# 一个字节的位值用数组表示，那么左边为高位。一个ic用数组表示，则左边为高字节。
# 下面要使用一个整体的十六进制来表示这个两字节的数据，而不是单独。
def can_data_2bytes(values):
    values = int(values * 1000)
    # print(values)
    low_byte = values & 0xFF  # 低字节
    high_byte = (values >> 8) & 0xFF  # 高字节
    return high_byte, low_byte
def can_data_2bytes_angle_10(values):
    values = int(values * 10)
    # print(values)
    low_byte = values & 0xFF  # 低字节
    high_byte = (values >> 8) & 0xFF  # 高字节
    return high_byte, low_byte
def can_data_2bytes_no_big(values):
    values = int(values)
    low_byte = values & 0xFF  # 低字节
    high_byte = (values >> 8) & 0xFF  # 高字节
    return high_byte, low_byte
def can_data_1bytes(values):
    values = int(values * 1000)
    low_byte = values & 0xFF  # 低字节
    return low_byte
def can_data_1bytes_no_big(values):
    values_ = int(values)
    # print(type(values_))
    low_byte = values_ & 0xFF # 低字节
    return low_byte

# '88 00 00 F0 00 12'
def get_hex(nums, frame):
    port_num = '88 00 00 00 8'
    port_frame = port_num + str(frame)
    for num in nums:
        hex_number = format(num, 'X')
        if len(hex_number) == 1:
            hex_number = '0' + hex_number
        port_frame += ' ' + str(hex_number)
    return port_frame
def get_bytes(data):
    can_data = bytes.fromhex(data)
    return can_data
def canID00_data(decimal_byte0, Failure, Type, MWRL, RearDistance):
    data = [0] * 8
    data[0] = can_data_1bytes_no_big(decimal_byte0)
    low0, high1 = can_data_2bytes_no_big(Failure)
    data[1], data[2] = low0, high1
    # data[1] = can_data_1bytes_no_big(decimal_MWR)
    # data[2] = can_data_1bytes_no_big(Failure)
    data[3] = can_data_1bytes_no_big(Type)
    low4, high5 = can_data_2bytes(MWRL)
    data[4], data[5] = low4, high5
    low6, high7 = can_data_2bytes(RearDistance)
    data[6], data[7] = low6, high7
    # print(data)
    data_hex = get_hex(data, 7)
    # print(data_hex)
    data_hex = data_hex[0:]
    print("00: " + data_hex)
    data_bytes = get_bytes(data_hex)
    # print(data_bytes)
    return data_bytes
def canID01_data(RearLeftToWall, RearRightToWall, ShovelPlateLeftToWall, ShovelPlateRightToWall):
    data = [0] * 8
    low0, high1 = can_data_2bytes(RearLeftToWall)
    data[0], data[1] = low0, high1
    low2, high3 = can_data_2bytes(RearRightToWall)
    data[2], data[3] = low2, high3
    low4, high5 = can_data_2bytes(ShovelPlateLeftToWall)
    data[4], data[5] = low4, high5
    low6, high7 = can_data_2bytes(ShovelPlateRightToWall)
    data[6], data[7] = low6, high7
    # print(data)
    data_hex = get_hex(data, 8)
    data_hex = data_hex[0:]
    print("01: " + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes
def canID02_data(CenterToLeftWall, CenterToRightWall, Car_z_Coordinate, CarbodyToRoof):
    data = [0] * 8
    low0, high1 = can_data_2bytes(CenterToLeftWall)
    data[0], data[1] = low0, high1
    low2, high3 = can_data_2bytes(CenterToRightWall)
    data[2], data[3] = low2, high3
    if Car_z_Coordinate < 0:
        Car_z_Coordinate *= (-1)
    low4, high5 = can_data_2bytes(Car_z_Coordinate)
    data[4], data[5] = low4, high5


    low6, high7 = can_data_2bytes(CarbodyToRoof)
    data[6], data[7] = low6, high7
    # print(data)
    data_hex = get_hex(data, 9)
    data_hex = data_hex[0:]
    data_bytes = get_bytes(data_hex)
    print("02: " + data_hex)
    return data_bytes
def canID03_data(Car_x_Coordinate, Car_y_Coordinate, RunningTime, MWRR):
    data = [0] * 8
    if Car_x_Coordinate < 0:
        low0, high1 = negativeAndPositive_2bytes(Car_x_Coordinate)
    else:
        low0, high1 = can_data_2bytes(Car_x_Coordinate)
    data[0], data[1] = low0, high1


    if Car_y_Coordinate < 0:
        low2, high3 = negativeAndPositive_2bytes(Car_y_Coordinate)
    else:
        low2, high3 = can_data_2bytes(Car_y_Coordinate)
    data[2], data[3] = low2, high3
    low6, high7 = can_data_2bytes_no_big(RunningTime)  # min
    data[6], data[7] = low6, high7

    low4, high5 = can_data_2bytes(MWRR)
    data[4], data[5] = low4, high5
    data_hex = get_hex(data, 'A')
    data_hex = data_hex[0:]
    print("03: " + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes
def canID04_data(Pitch, Yaw, decimal_ball, Version, Equipemt_number):
    data = [0] * 8
    if Pitch < 0:
        low0, high1 = negativeAndPositive_2bytes_angle(Pitch)      # x
    else:
        low0, high1 = can_data_2bytes_angle_10(Pitch)  # x
    data[0], data[1] = low0, high1
    low2, high3 = can_data_2bytes_angle_10(Yaw)
    data[2], data[3] = low2, high3

    data[4] = can_data_1bytes_no_big(decimal_ball)

    low5 = can_data_1bytes_no_big(Version)
    data[5] = low5
    low6, high7 = can_data_2bytes_no_big(Equipemt_number)
    data[6], data[7] = low6, high7
    # print(data)
    data_hex = get_hex(data, 'B')
    data_hex = data_hex[0:]
    print("04: " + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes


def canID05_data(LeftForwardMWR, LeftRearMWR, RightForwardMWR, RightRearMWR):
    data = [0] * 8

    low0, high1 = can_data_2bytes(LeftForwardMWR)
    data[0], data[1] = low0, high1
    low2, high3 = can_data_2bytes(LeftRearMWR)
    data[2], data[3] = low2, high3
    low4, high5 = can_data_2bytes(RightForwardMWR)
    data[4], data[5] = low4, high5
    low6, high7 = can_data_2bytes(RightRearMWR)
    data[6], data[7] = low6, high7
    # print(data)
    data_hex = get_hex(data, 5)
    data_hex = data_hex[0:]
    # print("05: " + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes

def canID06_data(LeftPointMWR, RightPointMWR, decimal_MWR, Failure):
    data = [0] * 8
    low0, high1 = can_data_2bytes(LeftPointMWR)
    data[0], data[1] = low0, high1
    low2, high3 = can_data_2bytes(RightPointMWR)
    data[2], data[3] = low2, high3

    data[4] = can_data_1bytes_no_big(decimal_MWR)
    data[5] = can_data_1bytes_no_big(Failure)

    # print(data)
    data_hex = get_hex(data, 6)
    data_hex = data_hex[0:]
    # print("06: " + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes


def can_data_2bytes_no_big_bigorder(values):
    values = int(values)
    high_byte = values & 0xFF # 高字节
    low_byte = (values >> 8) & 0xFF # 低字节
    return high_byte, low_byte

def canIDtest_data(a,b):
    data = [0] * 8

    high0, low1 = can_data_2bytes_no_big_bigorder(a)
    data[4], data[5] = high0, low1
    high2, low3 = can_data_2bytes_no_big_bigorder(b)
    data[6], data[7] = high2, low3

    print(data)
    data_hex = get_hex(data, 7)
    data_hex = data_hex[00:]
    print("07: " + data_hex)
    data_hex = '88 00 00 F0 07 46 82 03 E8 03 E8 03 E8'
    print('08: ' + data_hex)
    data_bytes = get_bytes(data_hex)
    return data_bytes



# 这里从can总线读取数据并解析，lidar和ins状态，
class ControlCommunication:
    def __init__(self, Version, Equipemt_number, model, Type, centertob1, hexahedron, roadporams):
        self.CenterToB1 = centertob1        # 机心在载体坐标，z设为0
        self.Hexahedron = hexahedron
        self.RoadParameter = roadporams
        self.Version = Version
        self.Equipemt_number = Equipemt_number
        self.model = model
        self.Type = Type
    def get_decimal_byte0(self, state, heatbeat, lidar_):
        # heatbeat = 0  # 心跳: 0--在线，1--不在线。
        # model = 0  # 模式: 0--调试，1正常。
        # state = '000'  # 状态：000：初始化。001：正常。010：故障。011：自检。100：关机。
        lidar1_state, lidar2_state, lidar3_state = self.get_lidar_state(lidar_)
        tmp = [heatbeat, self.model, state, lidar1_state, lidar2_state, lidar3_state]
        byte0 = ''
        for num in tmp:
            byte0 += str(num)
        decimal_byte0 = int(byte0[::-1], 2)
        return decimal_byte0
    # 距离以及一分钟看不到靶球
    def get_decimal_ball(self, balldist, minnotseeball, target_dist):
        if balldist < target_dist:
            dist_final = 0
        else:
            dist_final = 1
        # can_see_ball = 0
        if minnotseeball:  # 1分钟看不到靶球，则传给True进来，判断当前是否1分钟看不到靶球了。
            ball_state = 1
        else:
            ball_state = 0
        tmpball = [ball_state, 0, 0, 0, 0, 0, 0, 0]
        byte1 = ''
        for i in tmpball:
            byte1 += str(i)
        decimal_ball = int(byte1[::-1], 2)
        return decimal_ball

    def get_decimal_MWR(self, MWR_state):
        # MWR_state = self.get_MWR_state()  # 六个毫米波雷达状态：1正常，0异常。
        byte1 = ''
        for MWR in MWR_state:
            byte1 += str(MWR)
        decimal_MWR = int(byte1[::-1], 2)
        return decimal_MWR

    def get_data(self, car_coor, state, heatbeat, NeToRoad_angle, NeToIns_angle, RunningTime,
                 lidar_Failure, mwr_Failure, mwr_use, reardist, Lidar1_z, balldist, notseeballInMins, target_dist, x_a, z_a, lidar_state):
        T = Tunneling(car_coor[0][0], car_coor[1][0], car_coor[2][0], self.CenterToB1, NeToRoad_angle,
                      NeToIns_angle, self.Hexahedron, self.RoadParameter)   # 将坐标[]数组中的数值取出。注意报错

        # Forward_Dist = T.ForwardDistance(MWR_dist, HeadToMWR)
        RearDistance = T.RearDistance(reardist)
        ShovelPlateLeftToWall = T.ShovelPlateLeftToWall(x_a)
        ShovelPlateRightToWall = T.ShovelPlateRightToWall(x_a)
        RearLeftToWall = T.RearLeftToWall(x_a)
        RearRightToWall = T.RearRightToWall(x_a)
        CenterToLeftWall = T.CenterToLeftWall(x_a)
        CenterToRightWall = T.CenterToRightWall(x_a)
        CenterToCenterline = T.CenterToCenterline(x_a)
        CarbodyToRoof = T.CarbodyToRoof(Lidar1_z, z_a)
        Car_x_Coordinate = T.Car_x_Coordinate() + x_a

        Car_z_Coordinate = T.Car_z_Coordinate() - z_a
        Car_y_Coordinate = T.Car_y_Coordinate()
        yaw_pitch_roll = T.Yaw_Pitch_Roll()
        print(RearDistance, RearLeftToWall, RearRightToWall, ShovelPlateLeftToWall,
                ShovelPlateRightToWall, CenterToLeftWall, CenterToRightWall, CenterToCenterline,
                CarbodyToRoof, Car_x_Coordinate, Car_y_Coordinate, Car_z_Coordinate, yaw_pitch_roll)
        decimal_byte0 = self.get_decimal_byte0(state, heatbeat, lidar_state)
        mwr_data = self.get_six_mwr(mwr_use)
        MWR1, MWR2, MWR3, MWR4, MWR5, MWR6 = mwr_data[1][0], mwr_data[2][0], mwr_data[3][0], \
            mwr_data[4][0], mwr_data[5][0], mwr_data[6][0]
        MWR_s = [MWR1, MWR2, MWR3, MWR4, MWR5, MWR6, 0, 0]
        if mwr_use:
            print(MWR_s)

        decimal_MWR = self.get_decimal_MWR(MWR_s)
        MWRL, MWRR, MWRLF, MWRLR, MWRRF, MWRRR = mwr_data[1][1], mwr_data[2][1], mwr_data[3][1], \
            mwr_data[5][1], mwr_data[4][1], mwr_data[6][1]


        decimal_ball = self.get_decimal_ball(balldist, notseeballInMins, target_dist)


        # CAN数据
        data_canid_0 = canID00_data(decimal_byte0, lidar_Failure, self.Type, MWRL, RearDistance)
        data_canid_1 = canID01_data(RearLeftToWall, RearRightToWall, ShovelPlateLeftToWall, ShovelPlateRightToWall)
        data_canid_2 = canID02_data(CenterToLeftWall, CenterToRightWall, Car_z_Coordinate, CarbodyToRoof)
        data_canid_3 = canID03_data(Car_x_Coordinate, Car_y_Coordinate, RunningTime, MWRR)
        data_canid_4 = canID04_data(NeToIns_angle[1], NeToIns_angle[0], decimal_ball, self.Version, self.Equipemt_number)

        all_data = [data_canid_0, data_canid_1, data_canid_2, data_canid_3, data_canid_4]
        print(all_data)
        return all_data, Car_x_Coordinate

    # def send_data_can(self, data_can, can_id, time_interval):
    #     # 创建一个CAN总线接口
    #     bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)
    #     try:
    #         messages = []
    #         for idx, data in enumerate(data_can):
    #             message = can.Message(arbitration_id=can_id + idx, data=data, is_extended_id=True)
    #             messages.append(message)
    #
    #         for message in messages:
    #             bus.send(message)
    #             time.sleep(time_interval)
    #
    #         print("Data sent successfully!")
    #     except can.CanError as e:
    #         import traceback
    #         traceback.print_exc()
    #         print(f"Error sending data: {e}")
    #     finally:
    #         # 关闭CAN总线接口
    #         bus.shutdown()
    #arr = [b'\xe0\x01\x00\x01\x00\x00\x80>', b'f\x08\x94\x00\xa3\x06\xd4\xfe', b'\xad\r\xda\x05\x03\x0c\\\x0f', b'\xe9\x03\x8b\xef\x00\x00\x00\x00', b'\xfd\xff\x85\x03\x00\x01\x02\x00', b'\x00\x00\x00\x00\x00\x00\x00\x00', b'\x00\x00\x00\x00\x00\x02\x00\x00']
    def send_data_udp(self, data_can, time_interval):
        # 创建UDP套接字
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 目标IP和端口
        target_ip = '127.0.0.1'
        target_port = 20000

        # 将数组arr中的每个元素逐条发送
        for data in data_can:
            udp_socket.sendto(data, (target_ip, target_port))
            time.sleep(time_interval)
        print('udp_send_success')
        # 关闭套接字
        udp_socket.close()

    # 监控各个传感器状态，故障以及车身面临的情况。
    def check(self, params):
        pass
    # 这里得到的数据，是需要判断是否完成初始化，并且得到有用于计算的数据。
    def rece_data_from_can(self):  # 如果雷达有数据，则说明雷达初始化完成。
        pass
    def cal_mwr_dist(self):
        MWR_dist = 15
        return MWR_dist
    def get_lidar_state(self, lidar_state):
        lidar1 = lidar_state[0]
        lidar2 = lidar_state[1]
        lidar3 = lidar_state[2]
        return lidar1, lidar2, lidar3
    def get_six_mwr(self, mwr_is_available):
        if mwr_is_available:
            UDP_IP = "127.0.0.1"  # 根据实际情况修改IP地址
            UDP_PORT = 8000  # 根据实际情况修改端口号

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((UDP_IP, UDP_PORT))

            return_data = {}
            # {'1':[status, dist, reflection],'2':[status, dist, reflection]}
            while True:
                data, addr = sock.recvfrom(64)  # 接收8个字节的数据
                id = data[0]  # 第一个字节为id
                status = data[1]  # 第二个字节为状态，0表示掉线，1表示在线
                distance = int.from_bytes(data[2:6], byteorder='big')  # 第三到第六个字节为距离，使用big-endian字节序转换为整数
                reflection = int.from_bytes(data[6:8], byteorder='big')  # 最后两个字节为反射强度
                distance_m = distance / 10000.0

                if id not in return_data:
                    return_data[id] = [status, distance_m, reflection]

                if len(return_data) == 6:
                    break
        else:
            return_data = {1:[0, 0, 0], 2:[0, 0, 0], 3:[0, 0, 0], 4:[0, 0, 0], 5:[0, 0, 0], 6:[0, 0, 0]}
        return return_data

