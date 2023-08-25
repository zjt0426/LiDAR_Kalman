import time
import numpy as np
import math
# L--雷达；B--载体；N--东北天；H--巷道；I--惯导。


def setTrans(max_iter):
    """
    坐标转换主函数，完成两次坐标转换
    :param
    ------
    max_iter: 迭代次数
    :return:
    -------
    out: array
    """
    for i in range(max_iter):
        pos_b_l = LtoBtrans()
        pos_b_i = ItoHtrans()
    return pos_b_l, pos_b_i
def rotate_matrix(alpha):
    angle_radians = math.radians(alpha)
    # cos_value = math.cos(angle_radians)
    matrix_output = np.array([[np.cos(angle_radians), np.sin(angle_radians), 0],
                      [-np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])
    return matrix_output
def euler_matrix(angle):
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
    return np.dot(np.dot(A, B), C)

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
        C_i2n = euler_matrix(angle_ins[:, i][::-1])
        pos_n = np.dot(np.dot(C_i2n, C_b2i), pos_b)
        # 巷道坐标系和东北天的夹角，这里需要出入ZXY顺序。
        C_n2h = euler_matrix(angle_h_n[::-1]).T
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
        C_i2in = euler_matrix(angle_ins[:, i][::-1])
        pos_in0[:, i] = np.dot(C_i2in, pos_i1)
        # 载体相对于巷道当地测地
        pos_n_ins[:, i] = pos_in0[:, i] - pos_inh[:, i]
        # 载体原点相对于巷道坐标系
        C_n2h = euler_matrix(angle_h_n[::-1]).T
        pos_if[:, i] = np.dot(C_n2h, pos_n_ins[:, i])
        # 惯导速度和加速度转换到巷道坐标系
        vel_n[:, i] = np.dot(C_i2in, vel_i[:, i])
        acc_n[:, i] = np.dot(C_i2in, acc_i[:, i])

        vel_h[:, i] = np.dot(C_n2h, vel_n[:, i])
        acc_h[:, i] = np.dot(C_n2h, acc_n[:, i])

    return pos_if, vel_h, acc_h



# 卡尔曼滤波代码

def Kalman(x_last, P_last, A, H, cov_sta, cov_mea, out_ins, out_mwr):
    """
    卡尔曼滤波组合导航（MWR+INS）（毫米波雷达+惯导）
    状态方程：x_k+1 = A * x_k + noise_state
    测量方程：z_k = H * x_k + noise_measurment
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
    z = out_mwr - out_ins[0:3]
    z = z.reshape(3, 1)
    # print(z)
    # 1*3
    # 预测
    xpre = np.dot(A, x_last)
    # 12*1
    # print(xpre)
    Ppre = np.dot(np.dot(A, P_last), A.T) + cov_sta
    # 12*12

    # 校正
    XX = np.dot(H.T, np.linalg.inv(np.dot(np.dot(H, Ppre), H.T) + cov_mea))
    # print(XX.shape)
    Kal = np.dot(Ppre, XX)
    # print(Kal.shape)
    # print(np.dot(H, xpre))
    xpost = xpre + np.dot(Kal, (z - np.dot(H, xpre)))
    # print(xpost.shape)
    Ppost = Ppre - np.dot(np.dot(Kal, H), Ppre)
    # print(out_ins.shape, xpost.shape)
    # 结合INS数据，得到最终估计值
    est = out_ins.reshape(9, 1) + xpost[0:9]
    # print(out_ins)
    # print(xpost)
    # print(est.shape)
    # print(est)

    # 保存数据
    In_loop_solu = {'xpre': xpre, 'Ppre': Ppre, 'Kal': Kal, 'xpost': xpost, 'Ppost': Ppost, 'est': est}
    return In_loop_solu


if __name__ == "__main__":
    # angle_ins = []  # 读取惯导实时输出角度数据
    # max_iter = len(angle_ins[0])
    # 参数输入
    # 标定



    # 测试用
    import csv
    import numpy as np

    # 数据批量读取
    data_pre = []
    with open('final1.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data_pre.append(row)

    data_cur = data_pre[1:]
    data = [[0]*len(data_cur[0]) for _ in range(len(data_cur))]
    for i in range(len(data_cur)):
        for j in range(len(data_cur[0])):
            data[i][j] = float(data_cur[i][j])
    # 将数据转换为NumPy矩阵
    data = np.array(data)

    # data = data[0, :].reshape(-1, 21)

    # max_iter = 1
    max_iter = len(data)

    # 标定和坐标转换必要数据及参数
    angle_l_b = -3.1789
    angle_i_b = -0.4092
    angle_ins = data[:, 3:6].T
    angle_h_n = np.array([179.76375, 0.1000363, -0.127468879])   # 行向量
    out_lidar = data[:, 18:21].T
    # print(out_lidar)
    # print(out_lidar[:, 0].shape)
    # max_iter = len(angle_ins[0])

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
    print(pos_b_l)
    pos_b_i, vel_h_i, acc_h_i = ItoHtrans(jwg_ins, jwg_biaoba, max_iter, a, b, angle_i_b, angle_ins, angle_h_n,
              pos_ins, pos_if, pos_in0, pos_n_ins, pos_inh,
              vel_i, acc_i, vel_n, acc_n, vel_h, acc_h)
    # print(pos_b_i)
    out_ins = np.concatenate((pos_b_i, vel_h, acc_h), axis=0)
    print(out_ins)


    # 卡尔曼滤波准备
    alpha = -2.8485
    sigma = 0.5020
    tt = 0.6  # 最小时间处理单元
    var_mwr = 0.05
    # 状态误差的协方差
    cov_sta = tt ** 2 * np.block([[np.zeros((6, 12))],
                                  [np.zeros((3, 6)), sigma * sigma * np.eye(3), np.zeros((3, 3))],
                                  [np.zeros((3, 12))]])
    # print(cov_sta)
    # 测量误差的协方差（通过实验测定）
    cov_mea = var_mwr ** 2 * np.eye(3)
    # 状态转移矩阵
    aa = 1 + alpha * tt
    A = np.block([[np.eye(3), tt * np.eye(3), np.zeros((3, 6))],
                  [np.zeros((3, 3)), np.eye(3), tt * np.eye(3), tt * np.eye(3)],
                  [np.zeros((3, 6)), aa * np.eye(3), np.zeros((3, 3))],
                  [np.zeros((3, 9)), np.eye(3)]])
    # print(A)
    # 测量矩阵
    H = np.block([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    x0 = np.zeros((12, 1))
    P0 = 0.01 * np.eye(12)
    res = [0]*max_iter
    res[0] = {'xpre':[], 'Ppre':[], 'Kal':[], 'xpost':x0, 'Ppost':P0, 'est':np.array([])}
    # print(res)
    stable_number = max_iter

    for i in range(1, max_iter):
    # for i in range(1, 5):
        x_last = res[i-1]['xpost']
        P_last = res[i-1]['Ppost']
        if i > stable_number:
            last_ins = np.concatenate(res[i-1]['est'][0:6], out_ins[6:9, i-1], axis=0)
            x_last[0:6] = np.zeros((6, 1))
        else:
            last_ins = out_ins[:, i-1]
            # print(last_ins)

        res[i] = Kalman(x_last, P_last, A, H, cov_sta, cov_mea, out_ins[:, i], pos_lf[:, i])
    # print(res)


    # 单帧结果
    """
    index = 3
    kalman_est = []
    for i in range(index):
        kalman_est.append(res[i]['est'])
    print(kalman_est)
    """

    # 检查Ppost迹
    kf_est = np.zeros((9, max_iter))
    Ppost_trace = np.zeros(max_iter)
    for j in range(1, max_iter):
        kf_est[:, j] = np.squeeze(res[j]['est'], axis=(1, ))
        Ppost_trace[j] = np.trace(res[j]['Ppost'])

    # print(kf_est)
    # print(Ppost_trace)