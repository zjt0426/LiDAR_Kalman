# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import math

def rotate_matrix(alpha):
    angle_radians = math.radians(alpha)
    matrix_output = np.array([[np.cos(angle_radians), np.sin(angle_radians), 0],
                      [-np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])
    return matrix_output

def euler_matrix(angle, flag):
    # 这里的angle表示欧拉角的三个角度翻滚角、俯仰角以及偏航角。并且考虑正向YXZ输入顺序。
    # 规定惯导和东北天之间的夹角为：惯导输出的角度为东北天到惯导的角度。
    # flag :True--东北天到惯导    False--惯导到东北天角度


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


def DkeToEcef(x, y, z):
    """
    :功能描述: 将笛卡尔坐标系下的x、y、z转换为地心地固坐标系（WGS-84 LLA）下的经纬高。
    :param a: 地球为长半轴, 单位: cm
    :param b: 地球短半轴, 单位: cm
    :param x: x轴上的坐标
    :param y: y轴上的坐标
    :param z: z轴上的坐标
    :param e: 偏心率
    :param N:曲率半径
    :param epsilon：大于0的小常数
    :return: 地心地固坐标系下的坐标，经纬高

    """
    a = 6378137
    b = 6356752.31424
    epsilon = 0.000000000000001
    e = math.sqrt(math.pow(a, 2) - math.pow(b, 2)) / a
    lat = 0
    N = 0
    count = 0
    lat1 = np.arctan2(z, math.sqrt(math.pow(x, 2) + math.pow(y, 2)))

    while abs(lat - lat1) * 180 / math.pi > epsilon and count < 25:
        lat = lat1
        N = a / math.sqrt(1 - math.pow(e, 2) * math.pow(math.sin(lat), 2))
        lat1 = np.arctan2((z + N * math.pow(e, 2) * math.sin(lat)),
                          math.sqrt(math.pow(x, 2) + math.pow(y, 2)))
        count = count + 1

    x1 = np.arctan2(y, x) * 180 / math.pi
    y1 = lat * 180 / math.pi
    z1 = z / math.sin(lat) - N * (1 - math.pow(e, 2))

    return x1, y1, z1

def Htojwg(pos_if, jwg_biaoba, a, b, angle_i_b, angle_ins, angle_h_n,
              pos_ins, pos_in0, pos_n_ins, pos_inh,
              vel_i, acc_i, vel_n, acc_n, vel_h, acc_h):
    max_iter = 1
    for i in range(max_iter):
        C_n2h = euler_matrix(angle_h_n[::-1], flag=True)
        C_i2in = euler_matrix(angle_ins[:, i][::-1], flag=False)
        vel_n[:, i] = np.dot(C_n2h.T, vel_h[:, i])
        acc_n[:, i] = np.dot(C_n2h.T, acc_h[:, i])
        vel_i[:, i] = np.dot(C_i2in.T, vel_n[:, i])
        acc_i[:, i] = np.dot(C_i2in.T, acc_n[:, i])
        pos_n_ins[:, i] = np.dot(C_n2h.T, pos_if[:, i])
        C_i02i1 = rotate_matrix(angle_i_b)
        pos_i1 = -np.dot(C_i02i1, pos_ins)
        pos_in0[:, i] = np.dot(C_i2in, pos_i1)
        pos_inh[:, i] = pos_n_ins[:, i] - pos_in0[:, i]
        S = np.array([[-np.sin(math.radians(jwg_biaoba[0])), np.cos(math.radians(jwg_biaoba[0])), 0],
                      [-np.sin(math.radians(jwg_biaoba[1])) * np.cos(math.radians(jwg_biaoba[0])),
                       -np.sin(math.radians(jwg_biaoba[1])) * np.sin(math.radians(jwg_biaoba[0])),
                       np.cos(math.radians(jwg_biaoba[1]))],
                      [np.cos(math.radians(jwg_biaoba[1])) * np.cos(math.radians(jwg_biaoba[0])),
                       np.cos(math.radians(jwg_biaoba[1])) * np.sin(math.radians(jwg_biaoba[0])),
                       np.sin(math.radians(jwg_biaoba[1]))]])
        deltxyz = -np.dot(S.T, pos_inh[:, i])
        e_2 = ((a * a - b * b) / (a * a))
        N1 = a / ((1 - e_2 * np.sin(math.radians(jwg_biaoba[1])) ** 2) ** (1 / 2))
        x1 = (N1 + jwg_biaoba[2]) * np.cos(math.radians(jwg_biaoba[1])) * np.cos(math.radians(jwg_biaoba[0]))
        y1 = (N1 + jwg_biaoba[2]) * np.cos(math.radians(jwg_biaoba[1])) * np.sin(math.radians(jwg_biaoba[0]))
        z1 = (N1 * (1 - e_2) + jwg_biaoba[2]) * np.sin(math.radians(jwg_biaoba[1]))
        x0, y0, z0 = deltxyz + np.array([x1, y1, z1])
        xyz_ = DkeToEcef(x0, y0, z0)
        return xyz_,vel_i,acc_i


def cal_jwg(data,angle_ins,jwg_biaoba,angle_h_n):
    # 标定和坐标转换必要数据及参数
    angle_i_b = 0
    angle_ins = angle_ins.T

    # angle_h_n = np.array([0.15,	-0.2775,0.24])  # 行向量
    # jwg_biaoba = np.array([108.6788481, 34.2224881, 332.6])

    a = 6378137
    b = 6356752.31424
    pos_ins = np.array([0.09,  0.285,      0]).T  # 列向量
    pos_if = data[:,0:3].T
    pos_in0 = np.zeros((3, 1))
    pos_n_ins = np.zeros((3, 1))
    pos_inh = np.zeros((3, 1))
    vel_i = np.zeros((3, 1))
    acc_i = np.zeros((3, 1))
    vel_n = np.zeros((3, 1))
    acc_n = np.zeros((3, 1))
    vel_h =  data[:,3:6].T
    acc_h =  data[:,6:].T

    # 惯导坐标转换结果
    pos_b_i, vel_h_i, acc_h_i = Htojwg(pos_if, jwg_biaoba, a, b, angle_i_b, angle_ins, angle_h_n,
           pos_ins, pos_in0, pos_n_ins, pos_inh,
           vel_i, acc_i, vel_n, acc_n, vel_h, acc_h)
    return pos_b_i, vel_h_i, acc_h_i



# data = np.array([[1,1,1,0,0,-0.016,-0.0045,0.0073,0.0023]])
# angle_ins = np.array([[178.79,0.82,-0.29]])
# angle_h_n = np.array([179.76375, 0.1000363, -0.127468879])   # 行向量
# jwg_biaoba = np.array([1.086761300000000e+02, 34.240673100000000, 3.576238362000000e+02])
#
# pos_b_i, vel_h_i, acc_h_i = cal_jwg(data,angle_ins,jwg_biaoba,angle_h_n)
# print(pos_b_i, vel_h_i, acc_h_i)
