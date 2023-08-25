import time
import numpy as np
import math
import csv
# L--雷达；B--载体；N--东北天；H--巷道；I--惯导。

# 批量坐标转换代码


def rotate_matrix(alpha):
    angle_radians = math.radians(alpha)
    matrix_output = np.array([[np.cos(angle_radians), np.sin(angle_radians), 0],
                      [-np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])
    return matrix_output
def euler_matrix(angle, flag):
    # 这里的angle表示欧拉角的三个角度翻滚角、俯仰角以及偏航角。并且考虑正向YXZ输入顺序。
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

def LtoBtrans(angle_l_b, angle_i_b, angle_ins, angle_h_n, max_iter, out_lidar, pos_lidar, pos_lf):
    """
    雷达坐标转换：标靶-雷达-载体-惯导局部测地-巷道南
    :param
    ------
    angle_l_b: 载体x轴逆时针旋转到雷达x轴的角度
    angle_i_b: 载体x轴逆时针旋转到惯导x轴的角度
    angle_ins: 惯导实时输出和东北天的夹角（欧拉角）
    angle_h_n: 巷道和东北天的夹角
    ii: 当前迭代次数
    out_lidar: 标靶在雷达坐标系下的实时定位结果
    pos_lidar: 雷达原点在标靶坐标系坐标
    pos_lf: 3行L列的空矩阵，用来保存载体在雷达坐标系中坐标
    :return:
    -------
    out: array
    """
    for i in range(max_iter):
        C_l2b = rotate_matrix(angle_l_b).T
        pos_b = np.dot(C_l2b, out_lidar[:, i]) + pos_lidar
        C_b2i = rotate_matrix(angle_i_b)
        # 传入的惯导和东北天欧拉角的顺序是ZXY，这样取反解析的时候才会得到YXZC
        C_i2n = euler_matrix(angle_ins[:, i][::-1], flag=False)
        pos_n = np.dot(np.dot(C_i2n, C_b2i), pos_b)
        # 巷道坐标系和东北天的夹角，这里需要出入ZXY顺序。
        C_n2h = euler_matrix(angle_h_n[::-1], flag=True).T
        pos_h = np.dot(C_n2h, pos_n)
        # 保存数据
        pos_lf[:, i] = np.array([-pos_h[0], -pos_h[1], pos_h[2]])
    return pos_lf

def ItoHtrans(jwg_ins, jwg_biaoba, max_iter, a, b, angle_i_b, angle_ins, angle_h_n,
              pos_ins, pos_if, pos_in0, pos_n_ins, pos_inh,
              vel_i, acc_i, vel_n, acc_n, vel_h, acc_h):
    """
    惯导坐标转换：标靶-惯导+载体-惯导-----载体在巷道东北天-----载体在巷道坐标系
    :param
    ------
    jwg_ins: 惯导经纬高
    jwg_biaoba: 标靶经纬高
    ii: 迭代次数
    a: 地球长轴半径
    b: 地球短轴半径
    angle_i_b: 载体x轴绕逆时针旋转到惯导x轴的角度
    angle_ins: 惯导输出当前和东北天夹角
    angle_h_n: 巷道和东北天夹角
    pos_ins: 惯导原点在在载体坐标系下坐标
    pos_inh: 标靶在以惯导为原点的当地测地坐标系坐标
    pos_in0: 载体原点在惯导当地测地坐标
    pos_n_ins: 载体原点在巷道当地测地坐标系下坐标
    pos_if: 载体原点在巷道坐标系坐标
    vel_i: 惯导速度
    acc_i: 惯导加速度
    vel_n: 惯导东北天速度
    acc_n: 惯导东北天加速度
    vel_h: 惯导巷道速度
    acc_h: 惯导巷道加速度
    :return:
    -------
    out: array
    """
    # 注意这里python中下标是从0开始取的。
    for i in range(max_iter):
        e_2 = ((a*a - b*b)/(a*a))
        N0 = a/((1-e_2*np.sin(math.radians(jwg_ins[1, i]))**2)**(1/2))
        x0 = (N0+jwg_ins[2, i])*np.cos(math.radians(jwg_ins[1, i]))*np.cos(math.radians(jwg_ins[0, i]))
        y0 = (N0+jwg_ins[2, i])*np.cos(math.radians(jwg_ins[1, i]))*np.sin(math.radians(jwg_ins[0, i]))
        z0 = (N0*(1-e_2)+jwg_ins[2, i])*np.sin(math.radians(jwg_ins[1, i]))
        N1 = a/((1-e_2*np.sin(math.radians(jwg_biaoba[1]))**2)**(1/2))
        x1 = (N1+jwg_biaoba[2])*np.cos(math.radians(jwg_biaoba[1]))*np.cos(math.radians(jwg_biaoba[0]))
        y1 = (N1+jwg_biaoba[2])*np.cos(math.radians(jwg_biaoba[1]))*np.sin(math.radians(jwg_biaoba[0]))
        z1 = (N1*(1-e_2)+jwg_biaoba[2])*np.sin(math.radians(jwg_biaoba[1]))
        # 计算相对位置（标靶相对于惯导）
        deltxyz = np.array([x1-x0, y1-y0, z1-z0])
        S = np.array([[-np.sin(math.radians(jwg_ins[0, i])), np.cos(math.radians(jwg_ins[0, i])), 0],
                      [-np.sin(math.radians(jwg_ins[1, i]))*np.cos(math.radians(jwg_ins[0, i])), -np.sin(math.radians(jwg_ins[1, i]))*np.sin(math.radians(jwg_ins[0, i])),
                       np.cos(math.radians(jwg_ins[1, i]))],
                      [np.cos(math.radians(jwg_ins[1, i]))*np.cos(math.radians(jwg_ins[0, i])), np.cos(math.radians(jwg_ins[1, i]))*np.sin(math.radians(jwg_ins[0, i])),
                       np.sin(math.radians(jwg_ins[1, i]))]])
        pos_inh[:, i] = np.dot(S, deltxyz)
        # 载体相对于惯导
        C_i02i1 = rotate_matrix(angle_i_b)
        pos_i1 = -np.dot(C_i02i1, pos_ins)
        # 载体相对于惯导当地测地
        C_i2in = euler_matrix(angle_ins[:, i][::-1], flag=False)
        pos_in0[:, i] = np.dot(C_i2in, pos_i1)
        # 载体相对于巷道当地测地
        pos_n_ins[:, i] = pos_in0[:, i] - pos_inh[:, i]
        # 载体原点相对于巷道坐标系
        C_n2h = euler_matrix(angle_h_n[::-1], flag=True).T
        pos_if[:, i] = np.dot(C_n2h, pos_n_ins[:, i])
        # 惯导速度和加速度转换到巷道坐标系
        vel_n[:, i] = np.dot(C_i2in, vel_i[:, i])
        acc_n[:, i] = np.dot(C_i2in, acc_i[:, i])

        vel_h[:, i] = np.dot(C_n2h, vel_n[:, i])
        acc_h[:, i] = np.dot(C_n2h, acc_n[:, i])
    return pos_if, vel_h, acc_h

def dataloader(filename, single_list):
    data_pre = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data_pre.append(row)
    data_cur = data_pre[1:]
    data = [[0]*len(data_cur[0]) for _ in range(len(data_cur))]
    for i in range(len(data_cur)):
        for j in range(len(data_cur[0])):
            data[i][j] = float(data_cur[i][j])
    data = np.array(data)
    max_iter = len(data)
    # data = data[0, :].reshape(-1, 21)
    # max_iter = 1

    # 预留数组计算
    # data = np.array(single_list).reshape(-1, 21)
    return data, max_iter

def cal_output(data, max_iter):
    # 标定和坐标转换必要数据及参数
    angle_l_b = -3.1789
    angle_i_b = -0.4092
    angle_ins = data[:, 3:6].T
    angle_h_n = np.array([179.76375, 0.1000363, -0.127468879])   # 行向量
    out_lidar = data[:, 18:21].T
    pos_lidar = np.array([0.1239, -0.2469, 0]).T   # 列向量
    pos_lf = np.zeros((3, max_iter))



    jwg_ins = data[:, 0:3].T
    jwg_biaoba = np.array([1.086761300000000e+02, 34.240673100000000, 3.576238362000000e+02])
    a = 6378137
    b = 6356752.31424
    pos_ins = np.array([-0.1161, 0.2041, 0]).T  # 列向量
    pos_if = np.zeros((3, max_iter))
    pos_in0 = np.zeros((3, max_iter))
    pos_n_ins = np.zeros((3, max_iter))
    pos_inh = np.zeros((3, max_iter))
    vel_i = data[:, 9:12].T
    acc_i = data[:, 12:15].T
    vel_n = np.zeros((3, max_iter))
    acc_n = np.zeros((3, max_iter))
    vel_h = np.zeros((3, max_iter))
    acc_h = np.zeros((3, max_iter))
    # 雷达和惯导坐标转换结果
    pos_b_l = LtoBtrans(angle_l_b, angle_i_b, angle_ins, angle_h_n, max_iter, out_lidar, pos_lidar, pos_lf)
    pos_b_i, vel_h_i, acc_h_i = ItoHtrans(jwg_ins, jwg_biaoba, max_iter, a, b, angle_i_b, angle_ins, angle_h_n,
              pos_ins, pos_if, pos_in0, pos_n_ins, pos_inh,
              vel_i, acc_i, vel_n, acc_n, vel_h, acc_h)
    out_ins = np.concatenate((pos_b_i, vel_h, acc_h), axis=0)
    return pos_b_l, out_ins

if __name__ == '__main__':
    data, max_iter = dataloader('final1.csv', [])
    pos_b_l, out_ins = cal_output(data, max_iter)
    print(pos_b_l)
    print(out_ins)