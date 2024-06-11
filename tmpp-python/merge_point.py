# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd


def RotMat_N2Target(euler_angle_n2target):
    """
    由东北天到目标坐标系的旋转顺序：zxy（312）
    由目标坐标系到东北天的旋转顺序：yxz（213）
    采用角度制
    EulerAngle_N2Target (roll, pitch, yaw) （横滚角，俯仰角，偏航角） (y, x, z) (2, 1, 3)
    """

    roll = euler_angle_n2target[0]  # 横滚角roll  y 2
    pitch = euler_angle_n2target[1]  # 俯仰角pitch x 1
    yaw = euler_angle_n2target[2]  # 偏航角yaw   z 3

    cos_roll = np.cos(np.deg2rad(roll))
    sin_roll = np.sin(np.deg2rad(roll))

    cos_pitch = np.cos(np.deg2rad(pitch))
    sin_pitch = np.sin(np.deg2rad(pitch))

    cos_yaw = np.cos(np.deg2rad(yaw))
    sin_yaw = np.sin(np.deg2rad(yaw))

    RotMat_y = np.array([[cos_roll, 0, -sin_roll],
                         [0, 1, 0],
                         [sin_roll, 0, cos_roll]])

    RotMat_x = np.array([[1, 0, 0],
                         [0, cos_pitch, sin_pitch],
                         [0, -sin_pitch, cos_pitch]])

    RotMat_z = np.array([[cos_yaw, sin_yaw, 0],
                         [-sin_yaw, cos_yaw, 0],
                         [0, 0, 1]])

    rot_mat = np.dot(np.dot(RotMat_y, RotMat_x), RotMat_z)

    return rot_mat


def sphere_equation(point):
    # Sphere Equation:
    # x^2 + y^2 + z^2 + Ax + By + Cz + D = r^2;
    # Coefficient = [x, y, z, 1];
    # par = [A; B; C; D];
    # value = -(x^2 + y^2 + z^2);
    # Coefficient * par = value;

    coefficient = np.hstack((point.T, np.ones((point.shape[1], 1))))
    value = -np.sum(coefficient[:, :3] ** 2, axis=1)
    par = np.linalg.lstsq(coefficient.T.dot(coefficient), coefficient.T.dot(value), rcond=None)[0]
    center = -par[:3] / 2
    radius = np.sqrt(np.sum(center ** 2) - par[-1])

    return center, radius


def Point_Merge(pos_LidarPC_Lidar1, pos_LidarPC_Lidar2, pos_LidarPC_Lidar3, x, y):
    center, r = None, 0  # 设置默认值
    pos_LidarPC_LidarR = np.array([])
    angle_Car2Lidar_set = x
    pos_LidarO_Car = y
    # print(9999)
    # angle_Car2Lidar_set = np.array([[-0.81, -1.11, -0.04],
    #                                 [1.22, -2.67, 0.65],
    #                                 [-0.54, -1.86, 0.66]])
    #
    # pos_LidarO_Car = np.array([[-0.321, 0.533284072, 0.124499108],
    #                            [0.778, 0.533884376, 0.125042265],
    #                            [0.331, -0.248713655, 0.272606905]]).T


    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))

        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_LidarR2, pos_LidarPC_LidarR3), axis=1)
        center, r = sphere_equation(pos_LidarPC_LidarR)

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR3), axis=1)
        center, r = sphere_equation(pos_LidarPC_LidarR)

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR2), axis=1)
        center, r = sphere_equation(pos_LidarPC_LidarR)

    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = pos_LidarPC_LidarR3
        center, r = sphere_equation(pos_LidarPC_LidarR3)

    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = pos_LidarPC_LidarR2
        center, r = sphere_equation(pos_LidarPC_LidarR2)

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        pos_LidarPC_LidarR = pos_LidarPC_Lidar1
        center, r = sphere_equation(pos_LidarPC_Lidar1)

    if pos_LidarPC_Lidar1.shape[1] == 0 and pos_LidarPC_Lidar2.shape[1] == 0 and pos_LidarPC_Lidar3.shape[1] == 0:
        pos_LidarPC_LidarR = np.array([])
        center = None

    if pos_LidarPC_Lidar1.shape[1] != 0 and pos_LidarPC_Lidar2.shape[1] != 0 and pos_LidarPC_Lidar3.shape[1] != 0:
        # 雷达坐标系——载体坐标系
        angle_Car2Lidar2 = angle_Car2Lidar_set[1, :]
        RotMat_Car2Lidar2 = RotMat_N2Target(np.flip(angle_Car2Lidar2))
        pos_LidarPC_Car2 = pos_LidarO_Car[:, 1].reshape(3, 1) + np.dot(RotMat_Car2Lidar2.T, pos_LidarPC_Lidar2)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar2_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar2_ = RotMat_N2Target(np.flip(angle_Car2Lidar2_))
        pos_LidarPC_LidarR2 = np.dot(RotMat_Car2Lidar2_, (pos_LidarPC_Car2 - pos_LidarO_Car[:, 0].reshape(3, 1)))

        angle_Car2Lidar3 = angle_Car2Lidar_set[2, :]
        RotMat_Car2Lidar3 = RotMat_N2Target(np.flip(angle_Car2Lidar3))
        pos_LidarPC_Car3 = pos_LidarO_Car[:, 2].reshape(3, 1) + np.dot(RotMat_Car2Lidar3.T, pos_LidarPC_Lidar3)
        # （雷达点云）载体坐标系 → （雷达点云）雷达_参考坐标系2
        angle_Car2Lidar3_ = angle_Car2Lidar_set[0, :]
        RotMat_Car2Lidar3_ = RotMat_N2Target(np.flip(angle_Car2Lidar3_))
        pos_LidarPC_LidarR3 = np.dot(RotMat_Car2Lidar3_, (pos_LidarPC_Car3 - pos_LidarO_Car[:, 0].reshape(3, 1)))
        pos_LidarPC_LidarR = np.concatenate((pos_LidarPC_Lidar1, pos_LidarPC_LidarR2, pos_LidarPC_LidarR3), axis=1)
        # center, r = sphere_equation(pos_LidarPC_LidarR)

    return pos_LidarPC_LidarR


# data1= pd.read_excel(r'C:\Users\HP\Desktop\文件\02 掘进机精准定位\01 实验数据\激光雷达\8.18靶球静态-动态处理数据\动态\59.7cm\1\点云数据_帧数1_雷达1.xlsx').values.T
# data2=pd.read_excel(r'C:\Users\HP\Desktop\文件\02 掘进机精准定位\01 实验数据\激光雷达\8.18靶球静态-动态处理数据\动态\59.7cm\2\点云数据_帧数188_雷达2.xlsx').values.T
# data3=pd.read_excel(r'C:\Users\HP\Desktop\文件\02 掘进机精准定位\01 实验数据\激光雷达\8.18靶球静态-动态处理数据\动态\59.7cm\3\点云数据_帧数188_雷达3.xlsx').values.T
# print(data1)
# print(data2.shape[1])
# print(data3.shape[1])
# a = Point_Merge(data1,data2,data3)
# print(a)
