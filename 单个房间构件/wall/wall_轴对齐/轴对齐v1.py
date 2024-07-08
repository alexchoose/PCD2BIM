import open3d as o3d
import numpy as np
import os
from skimage.transform import (hough_line, hough_line_peaks)


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

def compute_normal(pcd,k=100):

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=k)
    )
    normals = np.array(pcd.normals)
    pcd_nor = np.mean(normals, axis=0)
    if abs(pcd_nor[0]) > abs(pcd_nor[1]):
        target_nor = np.array([1, 0, 0])
    else:
        target_nor = np.array([0, 1, 0])
    # pcd_nor[2] = 0
    return pcd_nor, target_nor

def wall_data_read(path):
    "读取墙体数据: Input : wall_data_path   Ouput: wall_data_list(列表格式)"
    wall_data_paths = os.listdir(path)
    wall_data = []
    for wall_data_path in wall_data_paths:
        temp = os.path.join(path,wall_data_path)
        wall_data.append(dataOpen(temp))
    return wall_data, wall_data_paths
def dataClean(point_cloud, distThres=0.2):
    '''
    输入一个open3d的点云格式文件，稍微过滤一些噪声数据、将该点云旋转至与轴平行的方向
    :param pcd: 输入点云
    :param distThres: 采用DBSCAN聚类时所需的参数，用于和其他房间点云区分，默认采用0.2因为一般隔墙0.2m
    :return: 输出pcd点云、旋转角度以及旋转中心，方便最后将输出结果旋转回原位置。输出根据轴对齐包围框筛选的点的索引
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
    downpcd = pcd.voxel_down_sample(voxel_size=0.01)   #参数voxel_size为体素大小，该参数越小，降采样得到的结果的点越多           # 先对点云采样提高计算效率  #体素下采样
    pData = np.asarray(downpcd.points)                                                                                # 获取xyz数据
    z_mean = np.mean(pData, axis=0)[2]
    index = np.where((pData[:, 2]<z_mean+0.5)&(pData[:, 2]>z_mean))[0]       #找出z轴上找到小于zmin+0.5大于zmin的         # 切片宽度0.05m
    slicePcd = downpcd.select_by_index(index)                             #将这数据全部取出                              # 选取切片

    #####################################
    # 用dbscan去除太远的离散数据
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            slicePcd.cluster_dbscan(eps=distThres, min_points=5, print_progress=True)).tolist()                       # 房屋0.2m

    maxlabel = max(labels, key=labels.count)
    ind = np.where(np.array(labels) == maxlabel)[0]                                                                   # 选择点数量最大的类别
    slicePcd = slicePcd.select_by_index(ind)
    obb = slicePcd.get_oriented_bounding_box()                                                                        # 获取切片数据的有向包围框
    sliceData = np.asarray(slicePcd.points)
    calData = sliceData - obb.center                                                                                  # 转化为局部坐标系
    angle = hough_line_detection(calData)                     #霍夫变换直线检测                                          # 采用HT检测直线获取最可靠直线对于坐标轴的夹角
    print(angle)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()#创建坐标框架网格的出厂函数。该坐标框架将以原点为中心。
    R = mesh.get_rotation_matrix_from_xyz((0, 0, angle*1.1))                                                             # 获得旋转角度对应的旋转矩阵（根据检测的直线夹角反向旋转）
    print(R)
    slicePcd.rotate(R, center=obb.center)
    pcd.rotate(R, center=obb.center)
    print(obb.center)
    aabb = slicePcd.get_axis_aligned_bounding_box()                                                                   # 获取切片数据的轴对齐包围框
    aabbPoints = np.asarray(aabb.get_box_points())
    aabb = o3d.geometry.PointCloud()
    aabb.points = o3d.utility.Vector3dVector(dataExpend(aabbPoints))                                                  # 将点云在z轴方向扩展并提取数据点
    aabb = aabb.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    pcd = pcd.crop(aabb)                                                                                              # 利用轴对齐包围框对点云裁剪
    # o3d.visualization.draw_geometries([pcd])
    return np.asarray(pcd.points), R
def hough_line_detection(data):
    img, _, _, _ = image_mapping(data, n=50)                                                                          # 点云映射至图像
    img = img[:, :, 0]
    img[np.nonzero(img)] = 1                                                                                          # 二值图像
    h, theta, d = hough_line(img)                                                                                     # HT直线检测
    accum, angle_o, dist_o = hough_line_peaks(h, theta, d)                                                            # 统计最高频次直线
    angle, count = np.unique(angle_o, return_counts=True)                                                             # 去除重复角度
    indx = np.argmax(count)                                                                                           # 选择最高频次的角度
    return angle[indx]
def image_mapping(data, n, delta=0.02):
    '''
    :param data: 输入的点云数据
    :param delta: 网格尺寸
    :param n: 图像扩增 一般50
    :return:
    '''
    dist = np.max(data[:, :3], axis=0) - np.min(data[:, :3], axis=0)       #最大最小值之间的差距的     注意dist为1×3        # 计算xyz轴方向的极差
    selectedCol = np.where(dist > np.min(dist))[0]                                                                    # 找到极差大的两个轴，用这两个轴的数据做图像映射
    indx_o = np.floor((data[:, selectedCol] - np.min(data[:, selectedCol], axis=0)) / delta + n).astype(np.int32)     # 直接根据xy坐标对切片数据进行    对输入的多维数组逐元素进行向下取整.
    indx, indx_arr = np.unique(indx_o, axis=0, return_index=True)    #index是返回的列表 index_arr是新列表元素在旧列表中的位置 # 将重复索引指标去除
    # 去除其中重复的元素 ，并按元素 由小到大 返回一个新的无元素重复的元组或者列表    return_index：如果为 true，返回新列表元素在旧列表中的位置（下标），并以列表形式存储
    points_sampled = data[indx_arr]
    img = np.zeros((np.max(indx[:, 0]) + n, np.max(indx[:, 1]) + n))                                                  # 创建图像空间, 增加n是为了让映射点云轮廓与边缘分开
    for i in range(indx.shape[0]):
        img[indx[i, 0], indx[i, 1]] = 255                                                                             # 网格内有数据点的为白色
    img = np.stack((img, img, img), axis=2).astype(np.uint8)                                                          # 将矩阵叠成RGB图片
    return img, indx, points_sampled, indx_o


def dataExpend(data, dist=0.05):
    '''
    将切片数据在z轴方向复制，从而获得超过楼层高度的轴向包围框
    :param data: 输入的切片数据
    :param dist: 包围框延伸的范围，这里默认在xy方向扩展0.05m因为最薄的隔墙一般0.1m
    :return:
    '''
    #####
    # 获取切片数据在xy平面内的4个角点
    x_min = np.min(data[:, 0])
    x_max = np.max(data[:, 0])
    y_min = np.min(data[:, 1])
    y_max = np.max(data[:, 1])
    x_min_id = np.where(np.around(data[:, 0], 4) == np.around(x_min, 4))[0]
    x_max_id = np.where(np.around(data[:, 0], 4) == np.around(x_max, 4))[0]
    y_min_id = np.where(np.around(data[:, 1], 4) == np.around(y_min, 4))[0]
    y_max_id = np.where(np.around(data[:, 1], 4) == np.around(y_max, 4))[0]
    #####
    # 包围框范围扩大一定范围
    data[x_min_id, 0] -= dist
    data[x_max_id, 0] += dist
    data[y_min_id, 1] -= dist
    data[y_max_id, 1] += dist
    #####
    # 在z轴方向上复制数据
    N = data.shape[0]
    expendData = np.zeros((3*N, 3))
    expendData[:N, :3] = data[:, :3]
    expendData[N:2*N, :2] = data[:, :2]
    expendData[N:2*N, -1] = data[:, 2] + 5                                                                            # 切片一般在房顶所以上方高度随便设置了5
    expendData[2*N:, :2] = data[:, :2]
    expendData[2*N:, -1] = data[:, 2] - 10                                                                            # 下方随便设置了10，只要保证超过扫描的高度范围就行
    return expendData
def rotate_point_cloud(point_cloud, R_temp):
    '''
    旋转矩阵，当前向量和目标向量
    :param pcd:
    :param source_vector:
    :param target_vector:
    :return:
    '''
    # 将向量归一化
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
    if np.all(R_temp == 0):
        source_vector, target_vector = compute_normal(pcd)
        # source_vector_normalized, target_vector_normalized = source_vector / np.linalg.norm(source_vector), target_vector / np.linalg.norm(target_vector)
        # 计算旋转轴和旋转角度
        axis = np.cross(source_vector, target_vector)
        angle = np.arccos(np.dot(source_vector, target_vector))
        # 应用旋转
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(-axis * angle*0.35)
    else:
        R = R_temp
    center = np.asarray([ -5.38315332, -0.39115823, 195.78747487])
    pcd.rotate(R, center = center)
    return np.array(pcd.points)

# 示例使用
if __name__ == '__main__':
    input_file_path = r'D:\ctz\code\PC_I\PCD2BIM_paper\wall_rh\data_wall'  # 输入点云文件路径
    wall_datas, wall_name = wall_data_read(input_file_path)
    R = np.asarray([[ 0.99705275,0.07671903, 0], [-0.07671903,0.99705275, 0], [ 0,0,1]])
    #输入整体房间进行R求解
    # for i in range(len(wall_name)):
    #     wall_point_cloud = wall_datas[i]
    #     temp_wall_all = np.vstack((temp_wall_all, wall_point_cloud))
    # _, R = dataClean(temp_wall_all)
    for i in range(len(wall_name)):
        aligned_point_cloud = rotate_point_cloud(wall_datas[i], R)
        np.savetxt(wall_name[i], aligned_point_cloud)
