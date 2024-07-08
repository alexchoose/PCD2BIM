import time
import open3d as o3d
import numpy as np
import os
import time
import matplotlib.pyplot as plt
import math
def dataOpen(path):
    '''
    :param path:读取文件路径
    :return: 输出点云
    '''

    if path.endswith('xyz') or path.endswith('txt'):
        if path.endswith('txt'):
            data = np.loadtxt(path)
        else:
            data = np.genfromtxt(path)
    elif path.endswith('npy'):
        data = np.load(path)
        _, M = np.shape(data)
        if M == 6:
            data = data[:, :3]
    elif path.endswith('ply'):
        pcd = o3d.io.read_point_cloud(path, format='ply')
        data = np.asarray(pcd.points)
    elif path.endswith('pcd'):
        pcd = o3d.io.read_point_cloud(path, format='pcd')
        data = np.asarray(pcd.points)
    return data[:,:3]

def element_data_read(path):
    "读取墙体数据: Input : wall_data_path   Ouput: wall_data_list(列表格式)"
    element_data_paths = os.listdir(path)
    element_data = []
    for wall_data_path in element_data_paths:
        temp = os.path.join(path,wall_data_path)
        element_data.append(dataOpen(temp))
    return element_data, element_data_paths

def main():
    input_wall_path = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐\wall_rh'
    input_window_path = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐\window'
    input_door_path = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐\door'

    #**************************************************数据读取部分**************************************************#
    walls_data, walls_name = element_data_read(input_wall_path)        #墙面数据读取
    windows_datas, windows_name = element_data_read(input_window_path) #window数据读取
    doors_datas, doors_name = element_data_read(input_door_path)       #door数据读取
    print(len(walls_name))
    print(len(windows_name))
    print(doors_name)



if __name__ == '__main__':
    main()
