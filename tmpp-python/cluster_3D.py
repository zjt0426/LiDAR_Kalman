# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN

def cluster_3D(data, label):
    # ymax = -5
    if label == 1:
        ymax = -0.5
        ymin = -10
        xmin = -2
        xmax = 2
        # 和俯仰角度有关
        zmin = -1
    elif label == 2:
        ymax = -1.5
        ymin = -10
        xmin = -1
        xmax = 3
        zmin = -1
    else:
        ymax = -1.5
        ymin = -10
        xmin = -3
        xmax = 1
        zmin = -1
    #print('0')
    #print(data)
    data = data[data['distance'] != 0]
    #data = data[data['x'] <= 4][data['x'] >= -4]
    data = data.loc[(data['x'] <= xmax) & (data['x'] >= xmin)]
    #print('1')
    #print(data)
    # data = data.loc[(data['z'] >= zmin)]
    # data = data[data['z'] > 0]
    #print('2')
    #print(data)
    data = data.loc[(data['y'] <= ymax) & (data['y'] >= ymin)]
    # data = data[data['y'] <= ymax][data['y'] >= -20]
    #print('3')
    #print(data)
    data = data[data['intensity'] < 100]
    data = data[data['intensity'] >= 98]
    #print('105----------')
    #print(data)
    data = data[['x','y','z','distance','frameId','intensity','frameTime','hAngle']]
    date = data[['frameTime']].drop_duplicates().values

    P = pd.DataFrame(np.random.randint(0, 1, size=(date.shape[0], 8)),
                     columns=['Time', 'center_x', 'center_y', 'center_z', 'distance_mean', 'intensity_mean', 'Laser_ID',
                              'label'])
    Q = pd.DataFrame(np.random.randint(0, 1, size=(date.shape[0], 5)),
                     columns=['Time', 'center_x', 'center_y', 'center_z', 'R'])
    result = pd.DataFrame()

    X = data
    if X.shape[0] > 3:
        db = DBSCAN(eps=0.3, min_samples=3).fit(X.iloc[:, :3])
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)  # 设置一个样本个数长度的全false向量
        core_samples_mask[db.core_sample_indices_] = True  # 将核心样本部分设置为true
        labels = db.labels_
        X['label'] = labels

        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        # 获取聚类个数。（聚类结果中-1表示没有聚类为离散点）

        unique_labels = set(labels)

        # 获取聚类簇的中心点
        d = np.ones([n_clusters_, 7])
        for k, i in zip(unique_labels, range(n_clusters_)):
            class_member_mask = (labels == k)
            xy = X.iloc[:, :7][class_member_mask & core_samples_mask]
            d[i] = xy.iloc[:, 0].mean(), xy.iloc[:, 1].mean(), xy.iloc[:, 2].mean(), xy.iloc[:, 3].mean(), xy.iloc[:,4].mean(), xy.iloc[:,5].mean(), int(k)

        columns = ['center_x', 'center_y', 'center_z', 'distance_mean', 'intensity_mean', 'Laser_ID', 'label']
        d = pd.DataFrame(d, columns=columns)
        d = d.sort_values(by='intensity_mean', ascending=False)
        if not d.empty:
            cluster_center = np.mat(d.iloc[0, :])
            P.iloc[0, :] = date[0][0], cluster_center[0, 0], cluster_center[0, 1], cluster_center[0, 2], cluster_center[
                0, 3], cluster_center[0, 4], cluster_center[0, 5], cluster_center[0, 6]

            label = P.iloc[0, -1]
            X = X[X['label'] == label]
            result = pd.concat([X, result], ignore_index=True)
            Point = result.iloc[:, :3]

        else:
            Point = result

    else:
        # df = pd.DataFrame()
        Point = result

    return Point


