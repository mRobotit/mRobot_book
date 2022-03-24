#!/usr/bin/env python
# coding=utf-8
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import sys
pd.set_option('display.float_format',lambda x : '%.7f' % x)

def get_media_pose(traj, time_stamp):
    right = traj.loc[traj['time_stamp']>=time_stamp].index
    if (len(right) <= 0 ):
        return pd.DataFrame(columns=traj.columns)
    right = right[0]
    left = right - 1
    if(right >= traj.shape[0] or left < 0):
        return pd.DataFrame(columns=traj.columns)
    right = traj.iloc[right]
    left = traj.iloc[left]
    weight_right = (time_stamp - left['time_stamp'])/(right['time_stamp'] - left['time_stamp'])
    weight_left = (right['time_stamp'] - time_stamp)/(right['time_stamp'] - left['time_stamp'])
    media = weight_right*right + weight_left*left
    return media

def tum_to_xytheta(pose):
    q = R.from_quat(pose[['qx', 'qy', 'qz', 'qw']].to_numpy())
    theta = q.as_euler('zxy')[0]
    ans = [pose['x'].item(), pose['y'].item(), theta[0]]
    return np.asarray(ans)

def xytheta_to_tum(pose, names=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']):
    q = R.from_euler('z', [pose[2]]).as_quat()
    pose = np.append([pose[0], pose[1], 1.0], q)
    pose = pd.DataFrame([pose.tolist()], columns=names)
    return pose

#由TUM格式的pose_i,pose_j计算得出它们之间的变换关系pose_ij
def get_pose_minus(pose_i, pose_j,names=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']):
    q_i = R.from_quat(pose_i[['qx', 'qy', 'qz', 'qw']].to_numpy())
    q_j = R.from_quat(pose_j[['qx', 'qy', 'qz', 'qw']].to_numpy())
    q_ij = q_i.inv()*q_j
    trans_i = np.array([pose_i['x'], pose_i['y']])
    trans_j = np.array([pose_j['x'], pose_j['y']])
    trans_ij = q_i.inv().as_matrix().dot(np.append((trans_j - trans_i), 1.0))
    pose_ij = np.append(trans_ij, q_ij.as_quat())
    pose_ij = pd.DataFrame([pose_ij.tolist()], columns = names)
    return pose_ij

def get_delta_result(traj_path, relation_path):
    tags = ['time_stamp','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    traj = pd.read_csv(traj_path, header = None, sep = " ", names=tags)
    ground_truth = pd.read_csv(relation_path, header = None, sep = " ")
    relations = ground_truth.to_numpy()
    delta = []
    for relation in relations:
        timestamp_i = relation[0]
        timestamp_j = relation[1]
        pose_i = get_media_pose(traj, timestamp_i)
        if(pose_i.shape[0] ==0 ):
            continue
        pose_j = get_media_pose(traj, timestamp_j)
        if(pose_j.shape[0] == 0):
            continue
        pose_ij = get_pose_minus(pose_i, pose_j)
        pose_ij_2d = tum_to_xytheta(pose_ij)
        pose_ij_ground = xytheta_to_tum([relation[2], relation[3], relation[7]])
        pose_ij_delta = get_pose_minus(pose_ij, pose_ij_ground)
        delta.append(np.linalg.norm(tum_to_xytheta(pose_ij_delta)))
    return delta


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Please input .txt(tum) and .relation(slam benchmarking ground truth)"
             "or .txt(tum) .txt(tum).... and .relation"
             )
        sys.exit()
    traj_paths = []
    for i in range(len(sys.argv)-2):
        traj_paths.append(sys.argv[1+i])
    relation_path = sys.argv[len(sys.argv)-1]
    
    deltas = []
    for traj_path in traj_paths:
        deltas.append(get_delta_result(traj_path, relation_path))
      
    plt.figure(figsize=(40, 4))
    for i in range(len(deltas)):
        delta = deltas[i]
        traj_path = traj_paths[i]
        x = np.arange(0, len(delta), 1)
        name = traj_path.split('.',1)[0]+" "
        plt.plot(x, delta, ls="-.", lw=1, label=name+r'$\delta T_{ij}$')
    plt.legend(loc="upper left")
    plt.savefig("compares.jpg")
    plt.show()
