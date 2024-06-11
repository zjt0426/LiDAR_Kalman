# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd


def myKalman(x_last, P_last, A, H, cov_sta, cov_mea, out_ins, out_mwr):
    """
    卡尔曼滤波组合导航（MWR+INS）
    状态方程： x_k+1 = A * x_k + noise_state
    测量方程：  z_k  = H * x_k + noise_measurment
    Input：
          x_last: 上一次误差状态矢量 12*1
          P_last: 上一次状态协方差 12*12
          A: 状态转移矩阵 12*12
          H: 测量矩阵 3*12
          cov_sta: 状态误差的协方差 12*12
          cov_mea: 测量误差的协方差 3*3
          out_ins: 单帧INS输出结果 9*1
          out_mwr: 单帧MWR定位结果 3*1
    Output:
          In_loop_solu: 内循环中的过程值 L维结构体
    """

    # 利用INS和MWR定位结果计算测量值
    z = out_mwr - out_ins[:3]

    # 预测
    xpre = np.dot(A, x_last)
    Ppre = np.dot(np.dot(A, P_last), A.T) + cov_sta

    # 校正
    XX = np.dot(H.T, np.linalg.inv(np.dot(np.dot(H, Ppre), H.T) + cov_mea))
    Kal = np.dot(Ppre, XX)  # 卡尔曼增益 12*3
    xpost = xpre + np.dot(Kal, (z - np.dot(H, xpre)))
    Ppost = Ppre - np.dot(np.dot(Kal, H), Ppre)

    # 结合INS数据，得到最终估计值
    est = out_ins + xpost[:9]

    # 保存数据
    In_loop_solu = {
        'xpre': xpre,
        'Ppre': Ppre,
        'Kal': Kal,
        'xpost': xpost,
        'Ppost': Ppost,
        'est': est
    }

    return In_loop_solu


def myKalman1(x_last, P_last, A, H, cov_sta,cov_mea, out_ins):
    """
    卡尔曼滤波组合导航（MWR+INS）
    状态方程： x_k+1 = A * x_k + noise_state
    测量方程：  z_k  = H * x_k + noise_measurment
    Input：
          x_last: 上一次误差状态矢量 12*1
          P_last: 上一次状态协方差 12*12
          A: 状态转移矩阵 12*12
          H: 测量矩阵 3*12
          cov_sta: 状态误差的协方差 12*12
          cov_mea: 测量误差的协方差 3*3
          out_ins: 单帧INS输出结果 9*1
          out_mwr: 单帧MWR定位结果 3*1
    Output:
          In_loop_solu: 内循环中的过程值 L维结构体
    """


    # 预测
    xpre = np.dot(A, x_last)
    Ppre = np.dot(np.dot(A, P_last), A.T) + cov_sta

    # 校正
    XX = np.dot(H.T, np.linalg.inv(np.dot(np.dot(H, Ppre), H.T) + cov_mea))
    Kal = np.dot(Ppre, XX)  # 卡尔曼增益 12*3
    xpost = xpre
    Ppost = Ppre - np.dot(np.dot(Kal, H), Ppre)

    # 结合INS数据，得到最终估计值
    est = out_ins + xpost[:9]

    # 保存数据
    In_loop_solu = {
        'xpre': xpre,
        'Ppre': Ppre,
        'Kal': Kal,
        'xpost': xpost,
        'Ppost': Ppost,
        'est': est
    }

    return In_loop_solu


# In_loop_solu = []  # 声明保存每次卡尔曼滤波迭代值的列表
# predict=[]
#
# # 第一个元素
# elem = {
#     'xpre': None,
#     'Ppre': None,
#     'Kal': None,
#     'xpost': np.zeros((12, 1)),
#     'Ppost': 0.01 * np.eye(12),
#     'est': None
# }
# In_loop_solu.append(elem)


def main(out_ins, pos_lf, In_loop_solu):
    alpha = -2.8485  # 时间相关系数
    tt = 0.05  # 时间单元
    sigma = 0.5020  # 高斯误差（标准差）
    var_mwr = 0.05  # 雷达定位误差（标准差）

    # 状态误差的协方差
    cov_sta = tt ** 2 * np.block([
        [np.zeros((6, 12))],
        [np.zeros((3, 6)), sigma ** 2 * np.eye(3), np.zeros((3, 3))],
        [np.zeros((3, 12))]
    ])

    # 测量误差的协方差
    cov_mea = var_mwr ** 2 * np.eye(3)

    # 状态转移矩阵
    aa = 1 + alpha * tt
    A = np.array([
        [1, 0, 0, tt, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, tt, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, tt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, tt, 0, 0, tt, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, tt, 0, 0, tt, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, tt, 0, 0, tt],
        [0, 0, 0, 0, 0, 0, aa, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, aa, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, aa, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
    ])
    # 测量矩阵
    H = np.concatenate((np.eye(3), np.zeros((3, 9))), axis=1)

    # 上一次状态矢量和状态协方差
    x_last = In_loop_solu[-1]['xpost']
    P_last = In_loop_solu[-1]['Ppost']

    if int(pos_lf.flatten()[0]) != 100000:
        # 利用卡尔曼滤波进行估计
        In_loop_solu.append(myKalman(x_last, P_last, A, H, cov_sta, cov_mea, out_ins.reshape(9, 1), pos_lf.reshape(3, 1)))
        #     predict.append(In_loop_solu[-1]['est'])

    if int(pos_lf.flatten()[0]) == 100000:
        In_loop_solu.append(myKalman1(x_last, P_last, A, H, cov_sta, cov_mea, out_ins.reshape(9, 1)))


    return In_loop_solu, In_loop_solu[-1]['est']

