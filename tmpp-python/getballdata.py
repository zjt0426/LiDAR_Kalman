import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from merge_point import Point_Merge
from PC3center import Centering_new


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


def cluster_3D1(data, clusternum):
    # result = pd.DataFrame()
    # result = np.empty(shape=(1, 4))
    X = data
    if X.shape[0] > 5:
        db = DBSCAN(eps=0.3, min_samples=5).fit(X[:, :3])
        # 获取每个类别的数量
        cluster_counts = pd.Series(db.labels_).value_counts()

        # 找到数量最多的三个类别（排除标签为-1的噪声点）
        n_clusters_ = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)

        # 计算聚类的数量
        clusternum = clusternum if n_clusters_ > clusternum else n_clusters_

        top_clusters = cluster_counts[cluster_counts.index != -1].nlargest(clusternum).index

        # 将数量最多的三个类别的数据提取出来组成新的DataFrame
        # Point = X[db.labels_.isin(top_clusters)]
        Point = X[np.isin(db.labels_, top_clusters)]
        # result = pd.concat([X, result], ignore_index=True)
        # Point = result.iloc[:, :4]
    else:
        Point = np.empty((0, 4))
    return Point


def cloud_point_axis(data, angle_Car2Lidar, angle_body_ins, angle_ins, angle_h_n, flag, pos_LidarO_Car, pos_Car0_Road,
                     distance_Ball_Road, Indentation, first_init):
    # data = data[data['distance'] != 0]
    # data = data.loc[(data['intensity'] > 1) & (data['intensity'] <= 120)]

    # 载体坐标系旋转到雷达坐标系的欧拉角
    angle_body_lidar = angle_Car2Lidar
    # 载体坐标系旋转到雷达坐标系的旋转矩阵
    Rotmat_body_lidar = RotMat_N2Target(angle_body_lidar[::-1])

    # 载体坐标系旋转到惯导坐标系的欧拉角
    angle_body_ins = angle_body_ins
    # 载体坐标系旋转到惯导坐标系的旋转矩阵
    Rotmat_body_ins = RotMat_N2Target(angle_body_ins)

    # 东北天坐标系旋转到惯导坐标系的欧拉角
    angle_enu_ins = angle_ins
    # 东北天坐标系旋转到惯导坐标系的旋转矩阵
    Rotmat_enu_ins = RotMat_N2Target(angle_enu_ins[::-1])

    # 东北天坐标系旋转到巷道坐标系的欧拉角
    angle_enu_road = angle_h_n
    # 东北天坐标系旋转到巷道坐标系的旋转矩阵
    Rotmat_enu_road = RotMat_N2Target(angle_enu_road[::-1])

    # 坐标转换
    # 1（标靶点云）雷达坐标系 → （标靶点云）（雷达）载体坐标系
    point_body = np.dot(Rotmat_body_lidar.T, data.T.iloc[0:3, :])
    # 2（标靶点云）（雷达）载体坐标系 → （标靶点云）（雷达）东北天坐标系
    point_enu = np.dot(np.dot(Rotmat_enu_ins.T, Rotmat_body_ins), point_body)
    # 3 3)（标靶点云）（雷达）东北天坐标系 → （标靶点云）（雷达）巷道坐标系
    point_road = np.dot(Rotmat_enu_road, point_enu)

    # 坐标筛选阈值设定
    if flag == 0:
        if first_init:
            x_min = -3  # -1.15  2.09  -6  0  -0.51  2
            x_max = 3
            y_min = -7
            y_max = -1
            z_min = -2
            z_max = 1.5
        else:
            x_min = -pos_Car0_Road[0] - distance_Ball_Road[0] + Indentation + 0.2  # -1.15  2.09  -6  0  -0.51  2
            x_max = -pos_Car0_Road[0] + distance_Ball_Road[1] - Indentation - 0.2
            y_min = -pos_Car0_Road[1] - Indentation - 0.5
            y_max = -pos_Car0_Road[1] + Indentation + 0.5
            z_min = -pos_Car0_Road[2] - distance_Ball_Road[3] + Indentation
            z_max = -pos_Car0_Road[2] + distance_Ball_Road[2] - Indentation
            print('flag==1,xyz-------')
            print(x_min, x_max, y_min, y_max, z_min, z_max)
    elif flag == 1:
        pos_Lidar2_Car_Road = np.dot(Rotmat_enu_road,
                                     np.dot(np.dot(Rotmat_enu_ins.T, Rotmat_body_ins), pos_LidarO_Car[:, 1]))
        pos_Lidar2_Road = pos_Lidar2_Car_Road + pos_Car0_Road
        if first_init:
            x_min = -3  # -1.15  2.09  -6  0  -0.51  2
            x_max = 3
            y_min = -7
            y_max = -1
            z_min = -2
            z_max = 1.5
        else:
            x_min = -pos_Lidar2_Road[0] - distance_Ball_Road[0] + Indentation + 0.2
            x_max = -pos_Lidar2_Road[0] + distance_Ball_Road[1] - Indentation - 0.2
            y_min = -pos_Lidar2_Road[1] - Indentation - 0.5
            y_max = -pos_Lidar2_Road[1] + Indentation + 0.5
            z_min = -pos_Lidar2_Road[2] - distance_Ball_Road[3] + Indentation
            z_max = -pos_Lidar2_Road[2] + distance_Ball_Road[2] - Indentation
            print('flag==2,xyz-------')
            print(x_min, x_max, y_min, y_max, z_min, z_max)
    else:
        pos_Lidar3_Car_Road = np.dot(Rotmat_enu_road,
                                     np.dot(np.dot(Rotmat_enu_ins.T, Rotmat_body_ins), pos_LidarO_Car[:, 2]))
        pos_Lidar3_Road = pos_Lidar3_Car_Road + pos_Car0_Road
        if first_init:
            x_min = -3  # -1.15  2.09  -6  0  -0.51  2
            x_max = 3
            y_min = -7
            y_max = -1
            z_min = -2
            z_max = 1.5
        else:
            x_min = -pos_Lidar3_Road[0] - distance_Ball_Road[0] + Indentation + 0.2
            x_max = -pos_Lidar3_Road[0] + distance_Ball_Road[1] - Indentation - 0.2
            y_min = -pos_Lidar3_Road[1] - Indentation - 0.5
            y_max = -pos_Lidar3_Road[1] + Indentation + 0.5
            z_min = -pos_Lidar3_Road[2] - distance_Ball_Road[3] + Indentation
            z_max = -pos_Lidar3_Road[2] + distance_Ball_Road[2] - Indentation
            print('flag==3,xyz-------')
            print(x_min, x_max, y_min, y_max, z_min, z_max)

    # 坐标筛选
    mask = (point_road[0] < x_max) & (point_road[0] > x_min) & (point_road[1] < y_max) & (point_road[1] > y_min) & (
            point_road[2] < z_max) & (point_road[2] > z_min)
    filtered_pos_point_Lidar = np.array(data.iloc[mask, :])
    # print(filtered_pos_point_Lidar)
    first_init = False
    return filtered_pos_point_Lidar, first_init


def curvature_line(points):
    column_means = np.mean(points, axis=0)
    points1 = points - column_means
    # print(points1)
    # 将点数据转置，以便进行SVD分解
    # points_T = points1.T
    # 对数据进行SVD分解
    U, S, Vt = np.linalg.svd(points1)
    # 获取最小特征值对应的特征向量
    normal = Vt[-1]
    a, b, c = normal
    # 计算平面到原点的距离
    d = -np.dot(normal, column_means)
    # 投影点到平面上生成新的点
    projected_points = points - (np.dot(points, normal) + d)[:, np.newaxis] * normal
    # 球心法
    return ball_center(projected_points)


def ball_center(Point):
    Point = Point[:, :3]
    # Point = np.unique(Point, axis=0)
    column_means = np.mean(Point, axis=0)
    Point = Point - column_means
    Coefficient = np.concatenate((Point, np.ones((Point.shape[0], 1))), axis=1)
    I = np.eye(Coefficient.shape[1])
    eps = 1e-6
    value = -np.sum(np.power(Coefficient[:, :3], 2), axis=1)
    par = np.linalg.solve(np.dot(Coefficient.T, Coefficient) + eps * I, np.dot(Coefficient.T, value))
    Center = -par[0:3] / 2
    Radius = np.sqrt(np.sum(Center ** 2) - par[-1])

    distances = sum(abs(np.linalg.norm(Point - Center, axis=1) - Radius)) / Point.shape[0]
    # print(f"行数{Point.shape[0]},距离{distances}")

    if distances > 0.035:
        Radius = 100
    return Radius, distances


def Compute_curvature(data, init_s, pos_Car0_Road):
    round_data = np.empty((0, 4))
    for laser_id in range(16):
        data_id = data[data[:, 3].astype(int) == laser_id]  # 雷达数据为空时，可能报错
        if data_id.shape[0] > 4:  # TODO
            X = data_id[:, :3]
            # if init_s:
            cluster_labels = DBSCAN(eps=0.12, min_samples=5).fit(X)  # TODO  5
            # else:
            #     print('eps------------')
            #     q = np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) * 0.007 + 0.05
            #
            #     cluster_labels = DBSCAN(eps=float(q[0]),
            #                             min_samples=5).fit(X)  # TODO  0.034

            labels = cluster_labels.labels_
            unique_labels = np.unique(labels)
            for label in unique_labels:
                if label != -1:
                    data_id_class = data_id[labels == label]
                    Point = data_id_class[:, :3]
                    Radius, distances = curvature_line(Point)
                    # curvature = 1.0 / Radius
                    if Radius < 0.22:  # TODO
                        round_data = np.vstack([round_data, data_id_class])
    return round_data


def extract_ball(point1, point2, point3, angle, coor, first_frame, init_s, pos_Car0_Road, centerk1, centerk0):
    if point1 is None:
        point1 = np.empty((0, 4))
    if point2 is None:
        point2 = np.empty((0, 4))
    if point3 is None:
        point3 = np.empty((0, 4))
    point1_coor = np.array(point1)[:, :3]
    point2_coor = np.array(point2)[:, :3]
    point3_coor = np.array(point3)[:, :3]

    pos_LidarPC_LidarR = Point_Merge(point1_coor.T, point2_coor.T, point3_coor.T, angle, coor).T

    insti1 = np.concatenate((point1[:, 3], point2[:, 3], point3[:, 3]), axis=0)
    data = np.column_stack((pos_LidarPC_LidarR, insti1))
    # 聚类
    X = data[:, :3]
    # print('x_shape')
    # print(X.shape[0])
    # X = np.concatenate((data[:, :3], data[:, 4][:, np.newaxis]), axis=1)
    if X.shape[0] > 4:
        if init_s:
            cluster_labels = DBSCAN(eps=0.2, min_samples=5).fit(X)  # TODO  5
        else:
            if np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) < 5:
                r = 0.16
            elif np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) < 6.4:
                r = 0.2
            elif np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) < 8:
                r = 0.25
            else:
                r = 0.3
            # r = 0.15 if np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) < 5 else 0.2
            cluster_labels = DBSCAN(eps=r, min_samples=5).fit(X)

            # r = 0.2 if np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) < 5 else 0.3
            # cluster_labels = DBSCAN(eps=np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) * 0.0255 + 0.02,
            # min_samples=5).fit(X)

            # q = np.linalg.norm(pos_Car0_Road.reshape(-1, 1), axis=0) * 0.0255 + 0.02
            # cluster_labels = DBSCAN(eps=float(q[0]),
            #                         min_samples=5).fit(X)  # TODO  0.034

        # 特征标准化
        # scaler = StandardScaler()
        # X = scaler.fit_transform(X)
        # cluster_labels = DBSCAN(eps=0.2, min_samples=5).fit(X)  # TODO  5
        labels = cluster_labels.labels_
        # print('cluster labels')
        # print(labels)
        labels_2d = np.reshape(labels, (-1, 1))
        curvature1 = np.concatenate((data, labels_2d), axis=1)

        # np.savetxt(r'D:\Desktop\xian0409\tmpp-xian\data71.csv', curvature1, delimiter=',')

        # print(labels)
        # 构建字典存储聚类结果
        cluster_dict = {}
        for i, label in enumerate(labels):
            if label == -1:  # 跳过噪声点
                continue
            if label not in cluster_dict:
                cluster_dict[label] = [data[i]]
            else:
                cluster_dict[label].append(data[i])
        label_single = -1
        label_double = -1
        error_double = 100
        distance_double = 100
        center_double = np.linalg.norm(centerk1 - centerk0)
        error_center_double = 1
        error_center_single = 1
        num_line = -1
        for label, cluster in cluster_dict.items():
            cluster = np.array(cluster)
            unique_values = np.unique(cluster[:, 3])
            num_unique_values = len(unique_values)
            # print('num_unique_values')
            # print(num_unique_values)
            if num_unique_values > 1:
                if cluster.shape[0] > 4:
                    if init_s:
                        line_list = [len(unique_values[(unique_values >= 0) & (unique_values <= 15)]),
                                     len(unique_values[(unique_values >= 16) & (unique_values <= 31)]),
                                     len(unique_values[(unique_values >= 32) & (unique_values <= 47)])]
                        if max(line_list) > 1:
                            max_index = line_list.index(max(line_list))
                            cluster = cluster[
                                (cluster[:, 3] >= max_index * 16) & (cluster[:, 3] <= (max_index + 1) * 16 - 1)]
                        Coefficient = np.concatenate((cluster[:, :3], np.ones((cluster.shape[0], 1))), axis=1)
                        I = np.eye(Coefficient.shape[1])
                        eps = 1e-6
                        value = -np.sum(np.power(Coefficient[:, :3], 2), axis=1)
                        par = np.linalg.solve(np.dot(Coefficient.T, Coefficient) + eps * I,
                                              np.dot(Coefficient.T, value))
                        Center = -par[0:3] / 2
                        Radius = np.sqrt(np.sum(Center ** 2) - par[-1])
                        distances = sum(abs(np.linalg.norm(cluster[:, :3] - Center, axis=1) - Radius)) / cluster.shape[
                            0]
                        # std_dev = np.std(abs(np.linalg.norm(cluster[:, :3] - Center, axis=1) - Radius))
                        if distances > 0.03:
                            Radius = 100
                        curvature = 1.0 / Radius
                        print(f"标签{label}，半径{Radius},距离{distances}，")
                        # if abs(curvature - 6.67) < 1 and abs(curvature - 6.67) < error_double and np.max(cluster[:, 0]) - np.min(cluster[:, 0]) < 0.4 and np.max(cluster[:, 1]) - np.min(cluster[:, 1]) < 0.4 and np.max(cluster[:, 2]) - np.min(cluster[:, 2]) < 0.4:
                        #     error_double = abs(curvature - 6.67)
                        #     label_double = label
                        if abs(curvature - 6.67) < 2 and distances < distance_double and np.max(
                                cluster[:, 0]) - np.min(
                                cluster[:, 0]) < 0.5 and np.max(cluster[:, 1]) - np.min(cluster[:, 1]) < 0.4 and np.max(
                            cluster[:, 2]) - np.min(cluster[:, 2]) < 0.5:
                            distance_double = distances
                            label_double = label
                            num_line = num_unique_values
                    else:
                        Coefficient = np.concatenate((cluster[:, :3], np.ones((cluster.shape[0], 1))), axis=1)
                        I = np.eye(Coefficient.shape[1])
                        eps = 1e-6
                        value = -np.sum(np.power(Coefficient[:, :3], 2), axis=1)
                        par = np.linalg.solve(np.dot(Coefficient.T, Coefficient) + eps * I,
                                              np.dot(Coefficient.T, value))
                        Center = -par[0:3] / 2
                        Radius = np.sqrt(np.sum(Center ** 2) - par[-1])
                        distances = sum(abs(np.linalg.norm(cluster[:, :3] - Center, axis=1) - Radius)) / \
                                    cluster[:, :3].shape[0]
                        if distances > 0.03:
                            Radius = 100
                        curvature = 1.0 / Radius
                        print(f"标签{label}，半径{Radius},距离{distances}，")
                        centerk0 = centerk0.reshape(-1)
                        distance_center = np.linalg.norm(centerk0 - Center)
                        # print('distence_center')
                        # print(distance_center)
                        if abs(curvature - 6.67) < 2 and abs(distance_center - center_double) < error_center_double and \
                                np.max(cluster[:, 0]) - np.min(cluster[:, 0]) < 0.5 and np.max(cluster[:, 1]) - \
                                np.min(cluster[:, 1]) < 0.4 and np.max(cluster[:, 2]) - \
                                np.min(cluster[:, 2]) < 0.5:
                            error_center_double = abs(distance_center - center_double)
                            label_double = label
                            num_line = num_unique_values
            else:
                Center = Centering_new(cluster[:, :3].T, 1)
                centerk0 = centerk0.reshape(-1)
                distance_center = np.linalg.norm(centerk0 - Center)
                if abs(distance_center - center_double) < error_center_single:
                    error_center_single = abs(distance_center - center_double)
                    label_single = label
        if label_double >= 0:
            print(f'最终标签{label_double}')
            return np.array(cluster_dict[label_double])[:, :3], num_line, first_frame
        elif label_single >= 0:
            return np.array(cluster_dict[label_single])[:, :3], 1, first_frame
        else:
            return np.array([]), 0, first_frame
    else:
        Point = np.array([])
        return Point, 0, first_frame


def get_ball_data(lidar1data, lidar2data, lidar3data, angle_Car2Lidar_set, angle_body_ins, angle_ins, angle_h_n,
                  pos_LidarO_Car, pos_Car0_Road, distance_Ball_Road, init_s, centerk1, centerk0, Indentation, maxclass):
    """
    靶球点云剖分：点云剖分坐标系-----聚类-----计算曲率-----融合三雷达数据识别靶球
    :param
    ------
    lidar1data: #1雷达采集的巷道数据
    lidar2data: #2雷达采集的巷道数据
    lidar3data: #3雷达采集的巷道数据
    angle_Car2Lidar_set：载体坐标系旋转到雷达坐标系的欧拉角
    angle_body_ins：载体坐标系旋转到惯导坐标系的欧拉角
    angle_ins：东北天坐标系旋转到惯导坐标系的欧拉角
    angle_h_n：东北天坐标系旋转到巷道坐标系的欧拉角
    pos_LidarO_Car:雷达原点在载体坐标系中的坐标
    pos_Car0_Road:载体原点在巷道坐标系中的坐标
    distance_Ball_Road:靶球中心距左壁、右壁、上壁、下壁的距离
    Indentation:巷道缩进距离
    maxclass:聚类最大类别

    :return:
    -------
    剖分点云: array
    剖分点云的线条数量
    """

    """
    数据格式：
        # 载体坐标系旋转到雷达坐标系的欧拉角
        angle_Car2Lidar_set = np.array([
            [0, 0, 0],
            [3.13, 7.96, 1.88],
            [-16.35, -4.68, -3.38]])  # 2.28, -1.08, -1.06  1.44, -1.57, -0.59
        # 载体坐标系旋转到惯导坐标系的欧拉角
        angle_body_ins = np.array([0, 0, 0])
        # 东北天坐标系旋转到巷道坐标系的欧拉角
        angle_h_n = np.array([90, 0, 0])
        # 雷达原点在载体坐标系中的坐标
        pos_LidarO_Car = np.array([
            [0, 0, 0],
            [-1.09396155, 1.30818361, -0.30573671],
            [1.04366303, 1.73805063, -0.33470241]
        ]).T
        # 载体原点在巷道坐标系中的坐标
        pos_Car0_Road = np.array(
            [-2.6, 9.578, -0.5]).T  # 0.2,2,-0.4     0.2,3.6,-0.4    1.28,3.45,-0.4   0.215,3.463,-0.5  -2.6,13.578,-0.5
        # 靶球中心距左壁、右壁、上壁、下壁的距离
        distance_Ball_Road = np.array(
            [1, 2, 2, 0.8]).T  # TODO 要考虑实际巷道的俯仰角  0.12    1.5,1.5,2,0.3     2,2,2,0.3   1.5,1,5,2,0.8  1,2,2,0.8
        Indentation = 0.2  # 缩进距离
        # 东北天坐标系旋转到惯导坐标系的欧拉角
        angle_ins = np.array([90, 0, 0])
    """

    # 读取原始点云数据
    Lidar1 = lidar1data[['x', 'y', 'z', 'id']]
    Lidar2 = lidar2data[['x', 'y', 'z', 'id']]
    Lidar3 = lidar3data[['x', 'y', 'z', 'id']]

    # 使用点云剖分坐标系过滤巷道壁的点云
    point1, state = cloud_point_axis(Lidar1, angle_Car2Lidar_set[0], angle_body_ins, angle_ins, angle_h_n, 0,
                                     pos_LidarO_Car, pos_Car0_Road, distance_Ball_Road, Indentation, init_s)
    point2, state = cloud_point_axis(Lidar2, angle_Car2Lidar_set[1], angle_body_ins, angle_ins, angle_h_n, 1,
                                     pos_LidarO_Car, pos_Car0_Road, distance_Ball_Road, Indentation, init_s)
    point3, state = cloud_point_axis(Lidar3, angle_Car2Lidar_set[2], angle_body_ins, angle_ins, angle_h_n, 2,
                                     pos_LidarO_Car, pos_Car0_Road, distance_Ball_Road, Indentation, init_s)
    # np.savetxt('point1.csv', point1, delimiter=',')
    # np.savetxt('point2.csv', point2, delimiter=',')
    # np.savetxt('point3.csv', point3, delimiter=',')

    # 对点云进行聚类，选取点数量最多的3-5类
    # point1 = cluster_3D1(Lidar1, maxclass)
    # point2 = cluster_3D1(Lidar2, maxclass)
    # point3 = cluster_3D1(Lidar3, maxclass)

    # 计算曲率，增加空的判断
    curvature1 = Compute_curvature(point1, init_s, pos_Car0_Road)
    curvature2 = Compute_curvature(point2, init_s, pos_Car0_Road)
    curvature2[:, 3] = curvature2[:, 3] + 16
    curvature3 = Compute_curvature(point3, init_s, pos_Car0_Road)
    curvature3[:, 3] = curvature3[:, 3] + 32

    return extract_ball(curvature1, curvature2, curvature3, angle_Car2Lidar_set, pos_LidarO_Car, state, init_s,
                        pos_Car0_Road, centerk1, centerk0)


