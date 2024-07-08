import open3d as o3d
import numpy as np
import numpy.matlib as npm
import pandas as pd
import os
import cv2
from skimage.transform import (hough_line, hough_line_peaks)
from sklearn.neighbors import KDTree
import random
from itertools import *
import argparse
import multiprocessing
import time

def dataOpen(path, savePath=''):
    '''
    :param path:读取文件路径
    :return: 输出点云 o3d格式
    '''

    filename = os.path.basename(path)
    suffix = filename.split('.')[-1]                         # 获取后缀名
    fname = filename.split('.')[0]                           # 获取文件名
    if suffix == 'xyz':
        data = np.genfromtxt(path)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data[:, :3])
        if data.shape[1] == 6:
            pcd.colors = o3d.utility.Vector3dVector(data[:, 3:])
        if savePath != '':
            pd.to_pickle(data, savePath+'/'+fname+'.pkl')
    elif suffix == 'asc':
        data = np.genfromtxt(path, skip_header=2)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data[:, :3])
        if data.shape[1] == 6:
            pcd.colors = o3d.utility.Vector3dVector(data[:, 3:])
        if savePath != '':
            pd.to_pickle(data, savePath + '/' + fname + '.pkl')
    elif suffix == 'npy':
        data = np.load(path)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data[:, :3])
        if data.shape[1] == 6:
            pcd.colors = o3d.utility.Vector3dVector(data[:, 3:])
        if savePath != '':
            pd.to_pickle(data, savePath + '/' + fname + '.pkl')
    elif suffix == 'pkl':
        data = pd.read_pickle(path)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data[:, :3])
        if data.shape[1] == 6:
            pcd.colors = o3d.utility.Vector3dVector(data[:, 3:]/255)
    else:
        pcd = o3d.io.read_point_cloud(path)
    return pcd

def dataClean(pcd, distThres=0.2):
    '''
    输入一个open3d的点云格式文件，稍微过滤一些噪声数据、将该点云旋转至与轴平行的方向
    :param pcd: 输入点云
    :param distThres: 采用DBSCAN聚类时所需的参数，用于和其他房间点云区分，默认采用0.2因为一般隔墙0.2m
    :return: 输出pcd点云、旋转角度以及旋转中心，方便最后将输出结果旋转回原位置。输出根据轴对齐包围框筛选的点的索引
    '''
    # o3d.visualization.draw_geometries([pcd])
    idx = np.arange(np.asarray(pcd.points).shape[0])    #asarray将点云数据变成数组形式，然后取出其第一维度值，变成数组
    idx = np.row_stack((idx, idx, idx)).T               #将其点云标号堆叠至Nx3的数组，然后翻转成3xN
    pcd.normals = o3d.utility.Vector3dVector(idx)       # From numpy to Open3D
    downpcd = pcd.voxel_down_sample(voxel_size=0.01)   #参数voxel_size为体素大小，该参数越小，降采样得到的结果的点越多           # 先对点云采样提高计算效率  #体素下采样
    center = downpcd.get_center()                                                                                     # 获取采样点中心，即旋转中心
    radiusFilterId = np.where(np.sum((np.asarray(downpcd.points)[:, :2]-center[:2])**2, axis=1) <= 200)[0]            # 经验性的删除扫描中心10米以外的点 #注意只用关心x和y轴不用关心z #np.where返回的是一个tuple，用[0]取出其中array
    downpcd = downpcd.select_by_index(radiusFilterId)
    pData = np.asarray(downpcd.points)                                                                                # 获取xyz数据
    hist, bins = np.histogram(pData[:, 2], bins=50)#bins代表区间 hist代表区间中的值 例如bins = [0,1] hist=[2],说明在0-1间有2个值 # 按照z轴进行分布数据分析
    bound = hist.argsort()[::-1][:2]#srgsort代表对hist升序排列，里面值代表下标，[::-1]代表翻转list，[:2]代表取前两个值           # 找到z轴上点分布最大的两个区间
    # z_min = np.mean(bins[bound])                                                                                    # 取这两个区间的均值作为切片的位置
    z_min = np.max(bins[bound]) -0.001                                                                        # 取这顶板下20cm作为切片的位置
    index = np.where((pData[:, 2]<z_min+0.05)&(pData[:, 2]>z_min))[0]       #找出z轴上找到小于zmin+0.05大于zmin的         # 切片宽度0.05m
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
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()#创建坐标框架网格的出厂函数。该坐标框架将以原点为中心。
    R = mesh.get_rotation_matrix_from_xyz((0, 0, -angle))                                                             # 获得旋转角度对应的旋转矩阵（根据检测的直线夹角反向旋转）
    slicePcd.rotate(R, center=obb.center)
    pcd.rotate(R, center=obb.center)
    aabb = slicePcd.get_axis_aligned_bounding_box()                                                                   # 获取切片数据的轴对齐包围框
    aabbPoints = np.asarray(aabb.get_box_points())
    aabb = o3d.geometry.PointCloud()
    aabb.points = o3d.utility.Vector3dVector(dataExpend(aabbPoints))                                                  # 将点云在z轴方向扩展并提取数据点
    aabb = aabb.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    pcd = pcd.crop(aabb)                                                                                              # 利用轴对齐包围框对点云裁剪
    # o3d.visualization.draw_geometries([pcd])
    return pcd, R, obb.center, np.asarray(pcd.normals)[:, 0]

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

def image_mapping_mean(plane, deviation, delta = 0.02):
    '''
    假设在小网格内数据的随机波动符合正态分布
    则将网格内的dist取平均值作为网格的偏差值
    :param plane: 点云
    :param deviation: 点到平面的距离
    :param delta: 网格尺寸
    :return: 输出图像以及每个点的在网格中的索引值
    '''

    dist = np.max(plane[:, :3], axis=0) - np.min(plane[:, :3], axis=0)
    selectedCol = np.where(dist > np.min(dist))[0]
    indx_o = np.floor((plane[:, selectedCol] - np.min(plane[:, selectedCol], axis=0)) / delta).astype(np.int32)
    ind = np.column_stack((indx_o, deviation.reshape(-1, 1)))                                                         # 将点到面的偏差与各点的索引对应
    indTemp = ind.tolist()                                                                                            # numpy转列表
    indTemp = sorted(indTemp, key=lambda x: (x[0], x[1]))                                                             # 将索引先按照第一维度排列再按第二维度排列
    indTemp = np.array(indTemp)
    N = np.max(indx_o[:, 0]) + 1                                                                                      # 避免整除情况
    M = np.max(indx_o[:, 1]) + 1
    img = np.zeros((N, M))
    ind_unique, unique_indices, unique_counts = np.unique(indTemp[:, :2], axis=0, return_index=True, return_counts=True) # 对索引进行去重
    ind_unique = ind_unique.astype(np.int32)
    for i in range(ind_unique.shape[0]-1):
        dist_temp = indTemp[unique_indices[i]:unique_indices[i+1], -1]                                                # 根据unique的索引值遍历取出数据
        if dist_temp.size != 0:
            img[ind_unique[i, 0], ind_unique[i, 1]] = np.mean(dist_temp)                                              # 属于同一个网格的偏差值取他们的均值作为该网格的偏差值（也可以取网格中的绝对偏差最大值，但设备的噪声无法避免）
    return img, indx_o

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

def plane_detection(pcd, rate=0.001):
    '''
    :param pcd: 输入点云
    :param rate: 当提取的平面数据超过多少后
    :return: 输出平面列表、平面模型的参数（法向量）、平面的点在原始数据中的索引
    '''
    data = np.asarray(pcd.points)                                                                                     # 取出点坐标
    num = data.shape[0]#num为点云数量
    plane_list = []
    plane_model_list = []
    idx_list = []
    while num > rate*data.shape[0]:                                                                                   # 多次使用RANSAC提取平面数据提取的距离阈值设置为0.02保证数据完整
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.012,
                                            ransac_n=3,
                                            num_iterations=1000)
        #distance_threshold 表示点到平面的阈值  返回平面与该平面的内点的序列号
        pcdPlane = pcd.select_by_index(inliers)                                                                       # 根据inliers提取平面点
        idx_list.append(np.asarray(pcdPlane.normals)[:, 0])                                                           # 将inliers索引存入列表
        largePlaneData = np.asarray(pcdPlane.points)                                                                  # 输出平面的点云数据
        plane_list.append(largePlaneData)                                                                             # 点云列表
        plane_model_list.append(plane_model)                                                                          # 平面的参数列表
        pcd = pcd.select_by_index(inliers, invert=True)                                                               # 翻转选择剩余的点云 invert = True代表选择除开inliers的点
        num = num - largePlaneData.shape[0]                                                                           # 剩余点数量
    return plane_list, plane_model_list, idx_list

def Area_filter(plane_list, areaSet, delta=0.02):
    '''
    对每一个表面进行面积检测，仅保留较大的平面，筛选掉较小的平面
    :param plane_list: 平面列表
    :param areaSet: 总面积限制
    :param delta: 网格尺寸
    :return: 输出筛选后的平面列表，筛选后平面的面积列表，筛选后的边缘列表，所选择的面的id， 所有表面的面积
    '''
    select_id = []
    AreaList = []
    boundaryList = []
    for i, each in enumerate(plane_list):                                                                             # 遍历平面
        img, idx, each_s, _ = image_mapping(each, n=50)                                                               # 将点云映射至平面
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                              # 转成灰度图像
        ret, bin_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)                           # 转为二值图像
        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)                         # 检测轮廓边缘
       #contours 表示轮廓本身  hierarchy表示轮廓类型          cv2.findContours 输入img 寻找轮廓模型 轮廓近似方式
        areaList = []                                                                                                 # 创建当前表面的面积列表
        cnt_id = []                                                                                                   # 轮廓像素点的id
        for j in range(len(contours)):
            area = cv2.contourArea(contours[j])                                                                       # 计算当前轮廓的图像面积
            areaList.append(area)                                                                                     # 将当前轮廓的所有图像面积添加入面积列表
        ind = np.where(np.array(areaList)*delta*delta > areaSet)[0]                                                   # 找出面积列表中实际面积比预设面积大的索引，为了排除点当前面中的小孔面积
        for k in range(ind.shape[0]):
            if k == 0:
                cnt = contours[np.int32(ind[k])]                                                                      # 仅有一个轮廓时就直接提取
            else:
                cnt = np.row_stack((cnt, contours[np.int32(ind[k])]))                                                 # 多个轮廓时，将所有轮廓索引合并
        cnt = np.squeeze(cnt)                                                                                         # 轮廓的数组为[n, 1, 2]需缩减为[n,2]
        try:
            for j in range(len(cnt)):                                                                                 # 尝试根据图中的轮廓找到对应的数据点
                id_c = np.where((idx[:, 1] == cnt[j, 0]) & (idx[:, 0] == cnt[j, 1]))[0]
                if id_c.size > 0:
                    cnt_id.append(id_c)

            if np.sum(np.array(areaList)[ind]) * delta * delta > areaSet:                                             # 该分割表面的总面积大于预设面积的就保留
                select_id.append(i)

            AreaList.append(np.sum(np.array(areaList)[ind]) * delta * delta)                                          # 总面积存入列表
            boundary_points = np.squeeze(each_s[np.asarray(cnt_id)])                                                  # 把粗略的边界点存入找出来存入列表
            boundaryList.append(boundary_points)
        except:
            AreaList.append(0)                                                                                        # 如果这个面的轮廓有问题，比如很破碎，那就直接面积为0不处理
            boundaryList.append([])
            continue
    if len(select_id)==0:                                                                                             # 如果一个面都不合格那就直接返回空
        return None, None, None, None, None
    else:
        return np.asarray(plane_list)[np.asarray(select_id)], np.asarray(AreaList)[np.asarray(select_id)], np.asarray(boundaryList)[np.asarray(select_id)], np.asarray(select_id), np.asarray(AreaList)

def FloorSlabSelection(plane_list, normalList):
    '''
    根据所有点的中心位置来确定天花板和地面
    :param plane_list: Ransac提取后的平面数据列表
    :param normalList: 法向量列表
    :return: 参考点；屋顶id，地面id
    '''
    center_list = []
    for each in plane_list:
        center_list.append(np.mean(each[:, :3], axis=0))                                                              # 获取表面列表中数据的中心
    center_list = np.array(center_list)                                                                               # 转成数组
    idMax = np.where(center_list[:, -1] == np.max(center_list[:, -1]))[0]#天花板                                       # 利用先验知识根据z轴找到这个方向的极值所在的平面
    idMin = np.where(center_list[:, -1] == np.min(center_list[:, -1]))[0]#地面
    horizontalPlaneId = np.where(np.around(np.dot(normalList[:, :3], normalList[idMax, :3].T)) == 1)[0]               # 根据法向量方向确定水平的平面索引
    #around 就是四舍五入
    return 0.5*(center_list[idMax]+center_list[idMin]), idMax[0], idMin[0], horizontalPlaneId

def candidates_extraction_for_DE(plane_list, boundary_list, r=0.1):
    '''
    根据边界点筛选出靠近边界的数据
    :param plane_list: 表面列表
    :param boundary_list: 边界点列表
    :param r: 离边界点的距离，默认为0.1m
    :return: 用于平面拟合的候选点列表
    '''
    candidateList = []
    for i in range(len(plane_list)):
        boundary = boundary_list[i]
        plane = plane_list[i]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(plane[:, :3])
        pcd_b = o3d.geometry.PointCloud()
        pcd_b.points = o3d.utility.Vector3dVector(boundary[:, :3])
        tree = o3d.geometry.KDTreeFlann(pcd)                                                                          # 将平面点云创建KDTreeFlann
        for j in range(len(boundary_list[i])):
            _, idx, _ = tree.search_radius_vector_3d(pcd_b.points[j], radius=r)                                       # 逐点提取邻域点
            if j==0:
                idx_all = np.asarray(idx)
            else:
                idx_all = np.append(idx_all, np.asarray(idx))
                idx_all = np.unique(idx_all)                                                                          # 去除重复点
        pcd_r = pcd.select_by_index(idx_all)
        pcd_c = pcd.select_by_index(idx_all, invert=True)                                                             # 将非靠近边缘点提取
        # o3d.visualization.draw_geometries([pcd_c])
        # o3d.visualization.draw_geometries([pcd_c])
        # o3d.visualization.draw_geometries([pcd_r])
        candidateList.append(np.asarray(pcd_c.points))                                                                # 将用于平面拟合的候选点添加至候选点列表
    return candidateList

def Normal_corresponding(plane_list, areaList, normalList):
    '''
    通过法向量将同一方向的平面分为一类，从而确定平行面关系
    :param plane_list: 表面点云列表
    :param areaList: 分割表面的面积列表
    :param normalList: 分割表面的法向量列表
    :return: 输出平行表面的点云，平行表面对应的面积，平行表面对应的法向量，平行表面的关系
    '''

    normals = np.abs(np.around(normalList[:, :3]))                                                                    # 将法向量归为xyz三个方向，且不区分方向性
    correspondingList = []
    for i in range(3):
        correspondingList.append(np.where(normals[:, i] == 1)[0])                                                     # 依次遍历xyz轴，将方向量一致的平面归为一类
        #因为法向量的平方和为1  所以当normals = 1 表明与该轴同方向
    correspondingPlane = []
    correspondingArea = []
    for each in correspondingList:                                                                                    # 遍历三个方向
        cp = []
        ca = []
        for i in each:                                                                                                # 将两两平行平面存进列表方便计算平面距离
            cp.append(plane_list[i])
            ca.append(areaList[i])
        correspondingArea.append(ca)                                                                                  # 分别存进平行平面的数据列表、面积列表、法向量列表
        correspondingPlane.append(cp)
    return correspondingPlane, correspondingArea, correspondingList

def PCA(data, sort=True):
    '''
    :param data: 输入数据
    :param sort: 降维排序
    :return: 输出特征值，特征向量，去中心化后的数据
    '''
    data_mean = np.mean(data, axis=0)                                                                                 # 对行求均值
    normalize_data = data - data_mean                                                                                 # 数据归一化操作
    H = np.dot(normalize_data.transpose(), normalize_data)                                                            # 协方差矩阵
    eigenvectors, eigenvalues, eigenvectors_transpose = np.linalg.svd(H)                                              # SVD分解
    if sort:                                                                                                          # 按照特征值从大到小排列
        sort = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:, sort]
    return eigenvalues, eigenvectors, normalize_data

def distanceCal(correspondingPlane, correspondingArea, countNum=10):
    '''
    根据平行表面的数据计算两个平行面的距离
    从面积小的平面上找随机点，并计算该点到另一个面上的最近邻点
    通过二者构成的向量和法向量方向判断这个距离是否能代表平行面距离
    :param correspondingPlane: 对应平行表面数据
    :param correspondingArea: 对应平行表面面积
    :param countNum: 每个平行面之间需要多少个测点，默认为10
    :return: 输出所有平行表面之间的测量距离
    '''
    distIndList = []
    for i in range(len(correspondingPlane)):
        planes = correspondingPlane[i]
        area = correspondingArea[i]
        comb = list(combinations(range(len(planes)), 2))                                                              # 将平行的表面进行组合
        #combination 从输入 iterable 返回元素的 r 长度子序列
        dists = []
        indices = []
        for each in comb:
            num1, num2 = each
            if area[num1] > area[num2]:                                                                               # 对当前组合对应的两个表面，判断表面的面积大小，以面积小的作为初始点选择平面
                idx, j = num1, num2                                                                                   # idx对应大面积表面，j为小面积表面
            else:
                idx, j = num2, num1
            point = planes[idx]                                                                                       # 将大面积平面建立KDTree
            tree = KDTree(point)
            indices_temp = []
            times = 0                                                                                                 # 防止陷入死循环的计数
            count = 0                                                                                                 # 测点计数
            while count<countNum:
                indx = np.random.randint(planes[j].shape[0], size=1)                                                  # 随机在小面积表面上选择一个点
                if indx not in indices_temp:                                                                          # 如果这个点没有被选过
                    dist, indi = tree.query(planes[j][indx], k=1)                                                     # 计算它到大表面点云上的最近邻点
                    #tree.query  找寻输入参数点 距离树最近的点 k = 1代表只找一个点 dist代表距离 indi代表该点在树中下标
                    indr = tree.query_radius(planes[idx][indi[0]], r=0.1)                                            # 将这个大表面点云上的最近邻点的邻域点取出计算局部法向量
                    #tree.query_radius 找寻该点的邻近 返回邻域的点的list
                    _, added_vectors, _ = PCA(planes[idx][indr[0]])   #这里PCA的意义在哪？
                    normal = added_vectors[:, -1]
                    direction = (planes[j][indx] - planes[idx][indi[0]])\
                                / np.linalg.norm(planes[j][indx] - planes[idx][indi[0]])                              # 两点连线的单位向量
                    linear = np.dot(normal.reshape(1, 3), direction.reshape(3, 1))                                    # 将单位向量与局部法向量做点积
                    #https://zhuanlan.zhihu.com/p/56541912  这篇文章解释了为什么用pca最后一列
                    if np.abs(np.abs(linear)-1)<0.00001:                                                              # 判断是否共线，阈值随便设置了一个0.00001
                        dists.append(dist[0])
                        d = {'OPId': idx, 'IPId': j, 'ONId': indi[0][0], 'INId': indx[0]}                             # 保存平面与测点信息
                        indices_temp.append(indx)
                        indices.append(d)
                        count += 1                                                                                    # 更新测点计数
                        times += 1                                                                                    # 更新计算次数
                    else:
                        times += 1                                                                                    # 换一个点，更新计算次数
                if times >= 100:                                                                                      # 计算超过100次停止
                    break
        distIndList.append([dists, indices])                                                                          # 将这个方向的平行面之间的距离存入列表
    return distIndList

def run_ransac(data, estimate, is_inlier, sample_size, goal_inliers, max_iterations, stop_at_goal=True, random_seed=None):
    '''
    RANSAC算法
    :param data: 输入数据
    :param estimate: 模型计算方法
    :param is_inlier: 判断内点方法
    :param sample_size: 采样数量
    :param goal_inliers: 目标内点数
    :param max_iterations: 最大迭代次数
    :param stop_at_goal: 达到目标是否停止
    :param random_seed: 随机种子是否固定
    :return:
    '''
    best_ic = 0
    best_model = None
    random.seed(random_seed)
    # random.sample cannot deal with "data" being a numpy array
    data = list(data)
    for i in range(max_iterations):
        s = random.sample(data, int(sample_size))
        m = estimate(s)
        ic = 0
        for j in range(len(data)):
            if is_inlier(m, data[j]):
                ic += 1
        if ic > best_ic:
            best_ic = ic
            best_model = m
            if ic > goal_inliers and stop_at_goal:
                break
    return best_model, best_ic

def augment(xyzs):
    '''
    :param xyzs: 输入数据，前三行xyz
    :return: [xyz1]n*4矩阵
    '''
    axyz = np.ones((len(xyzs), 4))
    axyz[:, :3] = xyzs
    return axyz

def estimate(xyzs):
    '''
    :param xyzs:输入数据，前三行是xyz
    :return: 平面参数
    '''
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    '''
    :param coeffs: 模型参数
    :param xyz: 输入点
    :param threshold: 距离阈值
    :return:
    '''
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

def plane_fitting_ransac_initial(data, per, ita):
    '''
    ax+by+cz+d = 0
    :param data: 输入数据
    :param per: 内点比例
    :param ita: 内点的距离阈值
    :return: 平面模型
    '''
    max_iterations = 1000
    goal_inliers = per * data.shape[0]
    m, _ = run_ransac(data, estimate, lambda x, y: is_inlier(x, y, ita), 3, goal_inliers, max_iterations)
    a, b, c, d = m
    plane = np.array([a/c, b/c, 1, d/c])
    return plane

def plane_fitting_ransac_area(data, per, reference_point):
    '''
    根据内点比例来确定最佳拟合平面
    :param data: 拟合数据
    :param per: 随机取出的数据占拟合数据中的百分比
    :param reference_point: 参考数据点
    :return: 拟合平面的参数
    '''
    ita = [0.01, 0.009, 0.008, 0.007, 0.006, 0.005, 0.004]                                                            # RANSAC的不同距离参数
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)
    downpcd = pcd.voxel_down_sample(voxel_size=0.02)                                                                  # 将数据轻量化以便于计算拟合平面
    data = np.asarray(downpcd.points)
    ###########################
    area_best = 1                                                                                                     # 初值设置为1，因为这里的判断准则是利用绝对距离超过0.003m的点作为不合格点（平整度范围8mm，绝对距离许可的算4mm吧）
    for each in ita:
        plane_initial_temp = plane_fitting_ransac_initial(data, 0.95, each)                          # 计算平面模型
        dist = np.dot(plane_initial_temp, np.column_stack((data[:, :3], np.ones((data.shape[0], 1)))).T) / np.sqrt(
            plane_initial_temp[0] ** 2 + plane_initial_temp[1] ** 2 + 1)                                              # 计算每个点到平面的距离
        pointId = np.where(np.abs(dist) >= 0.003)[0]                                                                  # 将绝对距离超过平面0.003m的点都取出
        area_best_temp = pointId.shape[0] / data.shape[0]                                                             # 计算外点比例
        if area_best_temp < area_best:                                                                                # 外点比例减少则储存平面模型
            # print(f'本次最佳比例：{area_best}')
            area_best = area_best_initial = area_best_temp
            plane_initial = plane_initial_temp
            # dist_best = dist
    # print('res_initial:{0}; maxDist:{1}'.format(area_best, np.max(np.abs(dist_best))))
    ###########################
    Res = np.zeros([201, 1])                                                                                          # 经过RANSAC计算初始模型后，计算200次随机采样拟合模型
    Num=0
    while Num <= len(Res) - 1:                                                                                        # RANSAC思想
        slice = random.sample(range(0, len(data[:, 0])), round(per * len(data[:, 0])))
        A = np.c_[data[slice, 0], data[slice, 1], np.ones(len(slice))]
        A1 = np.linalg.inv(np.dot(A.T, A))
        A2 = np.dot(A1, A.T)
        X = np.dot(A2, data[slice, 2])
        dist_temp = np.dot(np.array([X[0], X[1], -1, X[2]]), np.column_stack((data[:, :3], np.ones((data.shape[0], 1)))).T) / np.sqrt(
            X[0] ** 2 + X[1] ** 2 + 1)                                                                                # 计算点到面的距离
        pointId = np.where(np.abs(dist_temp) >= 0.003)[0]                                                             # 计算外点
        area = pointId.shape[0]/data.shape[0]                                                                         # 外点比例
        if area < area_best:                                                                                          # 保存最佳模型
            C_best = X
            area_best = area
            # dist_best = dist_temp
        Res[Num] = area_best
        Num += 1
    # print('res:{0}; maxDist:{1}'.format(area_best, np.max(np.abs(dist_best))))
    if area_best == area_best_initial:                                                                                # 如果迭代拟合的平面模型不佳，则输出初始化模型
        normal = plane_initial[:3]
        if normal.dot((reference_point - np.mean(data[:, :3], axis=0)).T) < 0:
            plane_initial = - plane_initial
        return plane_initial
    else:
        normal = np.array([C_best[0], C_best[1], -1]).reshape(1, 3)                                                   # 根据参考点大致确定平面法向量的方向
        if normal.dot((reference_point - np.mean(data[:, :3], axis=0)).T) > 0:
            plane = np.array([C_best[0], C_best[1], -1, C_best[2]])
        else:
            plane = -np.array([C_best[0], C_best[1], -1, C_best[2]])
        return plane

def deviationCal(candidates, boundary, plane, reference_point):
    '''
    :param candidates: 用于平面拟合的候选点（远离边缘点的数据点）
    :param plane: 表面点云数据
    :param reference_point: 参考中心
    :return: 输入所有点到面的距离，以及拟合平面法向量
    '''
    # 将表面点云建立KDTreeFlann便于根据边界点将边缘处的数据置为0
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(plane[:, :3])
    pcd_b = o3d.geometry.PointCloud()
    pcd_b.points = o3d.utility.Vector3dVector(boundary[:, :3])
    tree = o3d.geometry.KDTreeFlann(pcd)
    plane_coe = plane_fitting_ransac_area(candidates, 0.8, reference_point)                                         # 基于RANSAC思想的平面拟合方法
    dist = np.dot(plane_coe, np.column_stack((plane[:, :3], np.ones((plane.shape[0], 1)))).T) / np.sqrt(
        plane_coe[0] ** 2 + plane_coe[1] ** 2 + 1)                                                                  # 计算所有点到面的距离
    # 取出靠近边缘0.05米以内的点置为0
    for j in range(len(boundary)):
        _, idx, _ = tree.search_radius_vector_3d(pcd_b.points[j], radius=0.05)
        if j == 0:
            idx_all = np.asarray(idx)
        else:
            idx_all = np.append(idx_all, np.asarray(idx))
            idx_all = np.unique(idx_all)
    dist[idx_all] = 0
    normal = plane_coe[:3]
    normal = normal/np.linalg.norm(normal)
    return dist, normal

def windowSizeCal(theta, delta, ruleLength = 2):
    '''
    根据两米靠尺和图像网格尺寸
    计算滑动窗口的尺寸
    :param theta: 测量角度,角度制吧
    :param delta: 图像网格尺寸
    :param ruleLength: 靠尺长度，默认为2m
    :return: 滑动窗口高度和宽度
    '''

    w = np.ceil(ruleLength*np.sin(theta/360*2*np.pi)/delta)                                                           # 滑动窗口宽度尺寸计算
    h = np.ceil(ruleLength*np.cos(theta/360*2*np.pi)/delta)                                                           # 滑动窗口高度尺寸计算
    if w % 2 == 0:                                                                                                    # 保证为窗口宽和高为奇数
        w += 1
        w += 1
    if h % 2 == 0:
        h += 1
    return h, w

def elevation_difference_result_map_single(map, theta, delta):
    '''
    模拟人工靠尺计算平整度
    :param map: 输入的偏差图
    :return: 输出平整度图
    '''

    height, width = windowSizeCal(theta, delta)                                                                       # 根据输出角度设计窗口
    pad_img_2 = img_padding(map, np.int32((height-1)/2), np.int32((width-1)/2))                                       # 根据窗口进行图像扩展
    img = sliding_window_algorithm_diagonal(pad_img_2, height, width)                                                 # 计算平整度
    return img

def girdsSelect(img):
    '''
    :param img:窗口范围内涵盖的数据
    :return: 返回对角线所在的网格
    '''
    N, M = img.shape
    x = np.arange(N)
    y = np.around(x*(M-1)/(N-1))                                                                                      # 根据对角线斜率计算行所对应的列
    selectedGrid = np.row_stack((x, y)).T
    return selectedGrid

def sliding_window_algorithm_diagonal(img, height, width):
    '''
    sliding_window_algorithm 函数的改进版，只选择位于对角线上的点用来计算
    :param img: 输入偏差图
    :param height: 滑动窗口的高度
    :param width: 滑动窗口的宽度
    :return: 平整度计算图
    '''
    img_result = np.zeros_like(img)                                                                                   # 创建图像空间
    for i in range(np.int32((height-1)/2), np.int32(img.shape[0]-(height-1)/2)):
        for j in range(np.int32((width-1)/2), np.int32(img.shape[1]-(width-1)/2)):
            if height == width:
                window = img[np.int32(i - (height - 1) / 2):np.int32(i + (height - 1) / 2),
                         np.int32(j - (width - 1) / 2):np.int32(j + (width - 1) / 2)]                                 # 在输入图像中取出滑动窗口计算范围的值
                window_diagonal = np.diag(window)
            elif height == 1:
                window_diagonal = img[np.int32(i):np.int32(i+1), np.int32(j - (width - 1) / 2):np.int32(j + (width - 1) / 2)]  # 在输入图像中取出滑动窗口计算范围的值

            elif width == 1:
                window_diagonal = img[np.int32(i - (height - 1) / 2):np.int32(i + (height - 1) / 2), np.int32(j):np.int32(j+1)]  # 在输入图像中取出滑动窗口计算范围的值

            else:
                window = img[np.int32(i - (height - 1) / 2):np.int32(i + (height - 1) / 2),
                         np.int32(j - (width - 1) / 2):np.int32(j + (width - 1) / 2)]                                 # 在输入图像中取出滑动窗口计算范围的值
                selectedGrid = girdsSelect(window)
                window_diagonal = window[selectedGrid[:, 1], selectedGrid[:, 0]]
            img_result[i, j] = np.max(window_diagonal)-np.min(window_diagonal)                                        # 在窗口对角线数据上将最大值减最小值计算平整度
    return img_result[np.int32((height-1)/2):np.int32(img.shape[0]-(height-1)/2), np.int32((width-1)/2):np.int32(img.shape[1]-(width-1)/2)]


def img_padding(img, height, width):
    '''
    :param img: 输入图像
    :param height: 扩充高度，滑动窗口尺寸的一半
    :param width: 扩充宽度，滑动窗口尺寸的一边
    :return: 范围扩充后的图像
    '''
    return np.pad(img, ((height, height), (width, width)), 'constant')

def compute_flatness_single(plane, flatnessImg, ind, dist, threshold):
    '''
    根据每个点的索引从平整度图像中将平整度值赋给各点
    :param plane:输入点云
    :param img: 平整度图像
    :param ind: 每个点在图像中的索引
    :param dist: 每个点到平面的偏差
    :param threshold: 规范中规定的最大许可值
    :return: 返回各点的平整度具体值
    '''
    pc_num = len(plane)
    label = np.zeros((pc_num, 1))
    if flatnessImg.size != 0 or len(flatnessImg) != 0:
        # idx = np.where(deviationMap == 0)
        # flatnessImg[idx] = 0
        for j in range(pc_num):
            label[j] = flatnessImg[ind[j, 0], ind[j, 1]] * 1000
    label = np.clip(label, 0, threshold)                                                                              # 将超过阈值的计算结果置为最大值（这个要不要后面再看）
    label[dist == 0] = 0                                                                                              # 将靠近边界的点的平整度置零
    return label

def order_points(pts):
    '''
    :param pts: 轮廓坐标
    :return: 输出矩形的四个点
    '''
    # 列表中存储元素分别为左上角，右上角，右下角和左下角
    rect = np.zeros((4, 2), dtype = "float32")
    # 左上角的点具有最小的和，而右下角的点具有最大的和 (图的坐标系从y向下x向右)
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # 计算点之间的差值
    # 右上角的点具有最小的差值,
    # 左下角的点具有最大的差值
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # 返回排序坐标(依次为左上右上右下左下)
    return rect


def pointCreate(num, i, ptsList):
    '''
    根据3个点所在的位置确定第4个生成点
    :param num: 角的种类
    :param i: 计算角点
    :param ptsList: 角点列表
    :return: 生成的坐标点
    '''
    numbers = {
        0: np.array([np.max(ptsList[i-1:i+2, 0]), np.max(ptsList[i-1:i+2, 1])]),
        1: np.array([np.max(ptsList[i-1:i+2, 0]), np.min(ptsList[i-1:i+2, 1])]),
        2: np.array([np.min(ptsList[i-1:i+2, 0]), np.min(ptsList[i-1:i+2, 1])]),
        3: np.array([np.min(ptsList[i-1:i+2, 0]), np.max(ptsList[i-1:i+2, 1])])
    }
    return numbers.get(num)

def hull_order_points(approx, cnt):
    '''
    根据凸包获得的角点，找到角点间构成最大面积矩形的4个点的坐标
    输出角点的顺序分别为左上角，右上角，右下角和左下角
    :param pts: pts为提取的凸包角点坐标
    :return: 两个点
    '''
    # 输入角点顺序为逆时针旋转
    # 在首尾增加倒数1点和第1点
    pts = np.squeeze(approx)
    ptsList = np.row_stack((pts[-1].reshape(1, 2), pts, pts[0].reshape(1, 2)))
    rectArea = []
    rectList = []
    direction = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])/np.sqrt(2)                                             # 四个方向向量
    for i in range(1, ptsList.shape[0]-1):
        vector1 = (ptsList[i-1] - ptsList[i])/np.linalg.norm(ptsList[i-1] - ptsList[i])                               # 判断当前点与前后两点形成的向量组夹角方向
        vector2 = (ptsList[i+1] - ptsList[i])/np.linalg.norm(ptsList[i+1] - ptsList[i])
        vector = (vector1 + vector2)/np.linalg.norm(vector1 + vector2)
        tempVec = np.sqrt(np.sum((vector + direction)**2, axis=1))                                                    # 向量方向叠加就能确定角的方向
        directionId = np.where(tempVec == tempVec.max())[0]
        print('第{0}点向量内积为:{1}'.format(i, np.dot(vector1, vector2)))
        if np.abs(np.dot(vector1, vector2)) < 10e-2:                                                                  # 若当前点的角度为90，则判断生成的点是否在边界内
            ptTemp = pointCreate(directionId[0], i, ptsList)                                                          # 生成第3点
            manhatandist = np.sum(np.abs(cnt - ptTemp), axis=1)
            minId = np.where(manhatandist == np.min(manhatandist))[0]
            if minId.shape[0] > 1:
                ptTemp = cnt[minId[0]]
            else:
                ptTemp = cnt[minId[0]]
            # 对点云选择的4个角点排序，确定矩形的左上角和右下角端点
            rect = order_points(np.row_stack((ptsList[i-1:i+2], ptTemp.reshape(1, 2))))
            xs = [i[0] for i in rect]
            ys = [i[1] for i in rect]
            xs.sort()
            ys.sort()
            rectArea.append((xs[2] - xs[1]) * (ys[2] - ys[1]))
            rectList.append([xs[1], xs[2], ys[1], ys[2]])
        else:
            rectList.append(None)
            rectArea.append(0)
    selectId = np.where(np.array(rectArea) == max(rectArea))[0]                                                       # 选择面积最大的作为输出矩形
    return rectList[selectId[0]]

def ceilingDeviationByFivePoints(ceilingData, distance=0.25, delta=0.05):
    '''
    :param ceilingData: 顶板点云数据
    :param distance: 到边缘的距离（这里默认取0.25是因为之前有0.05cm的数据置零了）
    :param delta: 图像网格尺寸
    :return:
    '''

    img, _, samplePoint, mapInd = image_mapping(ceilingData, 1, delta=delta)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                                  # 映射图像转为灰度图
    _, thresh = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)                                  # 二值化
    cnts, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)                                          # 轮廓检测
    # img_copy = cv2.drawContours(img.copy(), cnts, -1, (0, 255, 0), 2)
    c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]                                                            # 找出最大的外轮廓点
    hull = cv2.convexHull(c, clockwise=True)                                                                          # 根据外轮廓找到包络
    # cv2.drawContours(img_copy, [hull], -1, (255, 255, 0), 2)
    hull = np.squeeze(hull)                                                                                           # 将[n, 1, 2]变成[n, 2]
    epsilon = 0.01 * cv2.arcLength(hull, True)                                                                        # 估计包络线弧长
    approx = cv2.approxPolyDP(hull, epsilon, True)                                                                    # 获得包络线顶点
    # pts = np.squeeze(approx)
    # for i in range(approx.shape[0]):
    #     img_copy = cv2.circle(img_copy, (int(pts[i, 0]), int(pts[i, 1])), 2, (0, 0, 255), 2)
    # cv2.imshow('rec', img_copy)
    # cv2.waitKey(0)
    rect = hull_order_points(approx, np.squeeze(c))
    xs = rect[:2]
    ys = rect[2:]
    rectResult = np.zeros((5, 2), dtype = "int32")                                                                    # 根据内接矩形的起点与终点计算5点的坐标
    dis = np.floor(distance/delta).astype(np.int32)
    rectResult[0] = np.array([xs[0], ys[0]]) + dis
    rectResult[1] = np.array([xs[0], ys[1]]) + dis * np.array([1, -1])
    rectResult[2] = np.array([xs[1], ys[1]]) + dis * np.array([-1, -1])
    rectResult[3] = np.array([xs[1], ys[0]]) + dis * np.array([-1, 1])
    rectResult[4] = np.floor(np.array([xs[0] + xs[1], ys[0] + ys[1]])/2)
    # 绘图
    img = cv2.rectangle(img, (int(xs[0]), int(ys[0])), (int(xs[1]), int(ys[1])), (255, 0, 0), 2)
    # cv2.imshow('五点法',img)
    for i in range(rectResult.shape[0]):
         img = cv2.circle(img, (rectResult[i, 0], rectResult[i, 1]), 2, (0, 0, 255), 2)
         print((rectResult[i, 0], rectResult[i, 1]))
         cv2.imshow('rec', img)
         cv2.waitKey(0)
    pointOutput = []
    for each in rectResult:
        print(gray_img[each[1], each[0]])
        if gray_img[each[1], each[0]] == 0:                                                                           # 判断图像网格是否有点
            each = np.array([each[1],each[0]])
            nonZerosId = np.array(np.where(gray_img != 0))
            manhatandist = np.sum(np.abs(np.array(nonZerosId).T-each), axis=1)
            minId = np.where(manhatandist == np.min(manhatandist))[0]                                                 # 曼哈顿距离最短的取出来
            if minId.shape[0] > 1:
                each = np.array(nonZerosId).T[minId[0]]
            else:
                each = np.array(nonZerosId).T[minId[0]]
            each = np.array([each[1], each[0]])
        tempId = np.where((mapInd[:, 0] == each[1]) & (mapInd[:, 1] == each[0]))[0]                                   # 找到对应网格中的点
        points = ceilingData[tempId]
        pointOutput.append(np.mean(points, axis=0).tolist())
    pts = np.array(pointOutput)
    sortId = np.argsort(pts[:, 2])                                                                                    # 根据z坐标排序
    pts = pts[sortId]
    result = np.around((pts[:, 2] - np.min(pts[:, 2]))*1000, 3)                                               # 输出结果
    return pts, result

def flatnessCal(i, candidates, plane, wallId, ceilingId, boundary, referencePoint, correspondingList, height, rotationMatrix, center, args):
    '''
    :param i: 第i个表面数据
    :param candidates: 平面拟合候选点
    :param plane: 表面点云
    :param boundary: 边界点
    :param wallId: 属于墙面的Id列表
    :param ceilingId: 属于顶板的Id列表
    :param referencePoint: 参考中心点
    :param correspondingList: 对应列表
    :param height: 墙面高度
    :param rotationMatrix: 墙面高度
    :param center: 墙面高度
    :param args:输入控制参数
    :return: 计算结果保存
    '''

    dist, normal = deviationCal(candidates, boundary, plane, referencePoint) #点到面的距离 及其法向量
    deviationMap, ind = image_mapping_mean(plane, dist, args.gridSize)
    flatnessImg = elevation_difference_result_map_single(deviationMap, args.rulerAngle, args.gridSize)
    flatness = compute_flatness_single(plane, flatnessImg, ind, dist, args.standardThreshold)
    # 根据平行关系在文件名上标出平行表面
    for pId, each in enumerate(correspondingList):
        if i in each:
            break
    if pId == 0:
        pIdresult = 'A'
    elif pId == 1:
        pIdresult = 'B'
    else:
        pIdresult = 'C'
    # 判断数据的属性
    if i in wallId:
        theta = np.abs(np.arccos(np.dot(np.array([0, 0, 1]), normal)) - np.pi / 2)                                    # 根据z轴坐标计算垂直度
        deviation = height * np.tan(theta) * 1000
        verticality = deviation
        outputPath = os.path.join(args.outputPath,
                                  args.fileName.split('.')[0] + '_wall_{0}_{1}.pcd'.format(i, pIdresult))
    elif i == ceilingId:
        verticality = 0
        outputPath = os.path.join(args.outputPath,
                                  args.fileName.split('.')[0] + '_ceiling_{0}_{1}.pcd'.format(i, pIdresult))
    else:
        verticality = 0
        outputPath = os.path.join(args.outputPath,
                                  args.fileName.split('.')[0] + '_floor_{0}_{1}.pcd'.format(i, pIdresult))
    planePcd = o3d.geometry.PointCloud()
    planePcd.points = o3d.utility.Vector3dVector(plane)
    planePcd.rotate(np.linalg.inv(rotationMatrix), center=center)
    planePcd.normals = o3d.utility.Vector3dVector(np.column_stack(
        (dist.reshape(-1, 1) * 1000, flatness.reshape(-1, 1), np.ones_like(dist.reshape(-1, 1)) * verticality)))     # 将偏差、平整度、垂直度放在pcd中的normal输出
    # samplePcd = planePcd.voxel_down_sample(0.02)
    step = 1/args.sampleRate                                                                                         # 输出结果采样比例，当为0.5时，相当于间隔步长为1/0.5=2
    index = np.arange(0, plane.shape[0], step)
    index = np.unique(np.floor(index).astype(np.int32))
    samplePcd = planePcd.select_by_index(index)
    o3d.io.write_point_cloud(outputPath, samplePcd)
    print('已输出第{0}平面的平整度与垂直度结果'.format(i))


def parse_args():
    parser = argparse.ArgumentParser('Path')
    parser.add_argument('--fileName', type=str, default='a-2.pcd', help='input data name')
    parser.add_argument('--inputPath', type=str, default='./data/', help='input data path')
    parser.add_argument('--outputPath', type=str, default='./data/output/a-2pcd', help='output data path')
    parser.add_argument('--sampleRate', default=1.0, type=float, help='data sampling rate')
    parser.add_argument('--pointSavePath', type=str, default='txt', help='xyz or txt can be saved as pkl')
    parser.add_argument('--countNum', default=10.0, type=float, help='number of measurements for spatial dimension estimation')
    parser.add_argument('--areaThres', default=1.0, type=float, help='filtering thres using area')
    parser.add_argument('--distThres', default=0.2, type=float, help='thres for room extraction')
    parser.add_argument('--rate', default=0.001, type=float, help='ratio of the number of termination points of plane segmentation')
    parser.add_argument('--gridSize', default=0.02, type=float, help='size of mapped image')
    parser.add_argument('--distToBoundary', default=0.1, type=float, help='distance to boundary')
    parser.add_argument('--rulerAngle', default=45, type=float, help='angle of placed ruler')                           # 可设置为多个角度，不过代码得相应修改代码，这个还没写，不过很容易
    parser.add_argument('--standardThreshold', default=8, type=float, help='limit specified by code')
    parser.add_argument('--referencePoint', default=True, type=bool, help='Whether (0,0,0) is the center of pcd')
    parser.add_argument('--ceilingDistToBoundary', default=0.25, type=float, help='distance to boundary for ceiling')
    return parser.parse_args()

def main(args):
    # result = []
    dataPath = os.path.join(args.inputPath, args.fileName)
    pcd = dataOpen(dataPath, args.pointSavePath)
    #1
    pcd, rotationMatrix, center, firstSelectedId = dataClean(pcd, args.distThres)    #滤波操作
    ####################################
     # 平面分割提取各表面并区分顶板、底板以及墙面
    planeList, normalList, inliersIdList = plane_detection(pcd, args.rate)          #检测平面       返回：输出平面列表、平面模型的参数（法向量）、平面的点在原始数据中的索引
    planeListFilter, areaList, boundaryList, selectedId, areaListAll = Area_filter(planeList, args.areaThres,
                                                                                   args.gridSize)  #通过面积筛选平面  返回：筛选后的平面列表，筛选后平面的面积列表，筛选后的边缘列表，所选择的面的id， 所有表面的面积
    normalList = np.array(normalList)[selectedId]
    inliersIdList = np.array(inliersIdList)[selectedId]
    referencePoint, ceilingId, floorId, horzontalPlaneId = FloorSlabSelection(planeListFilter, normalList)#根据所有点的中心位置来确定天花板和地面      返回： 参考点；屋顶id，地面id
    if args.referencePoint is True:
        referencePoint = np.array([0, 0, 0])
    # 开间进深
    candidateList = candidates_extraction_for_DE(planeListFilter, boundaryList, args.distToBoundary)#根据边界点筛选出靠近边界的数据 返回：用于平面拟合的候选点列表
    correspondingPlane, correspondingArea, correspondingList = Normal_corresponding(candidateList, areaList, normalList)#通过法向量将同一方向的平面分为一类，从而确定平行面关系 返回：输出平行表面的点云，平行表面对应的面积，平行表面对应的法向量，平行表面的关系
    distIndList = distanceCal(correspondingPlane, correspondingArea, args.countNum)# 根据平行表面的数据计算两个平行面的距离  返回：输出所有平行表面之间的测量距离
    #####################################
    # 取出空间尺寸的测点位置
    newId = 0
    for i in range(len(distIndList)):
        dist, insId = distIndList[i][0], distIndList[i][1]
        for j in range(len(insId)):
            opId, ipId, onId, inId = insId[j]['OPId'], insId[j]['IPId'], insId[j]['ONId'], insId[j]['INId']
            if newId == 0:
                new = np.column_stack((correspondingPlane[i][opId][onId].reshape(1, 3),
                                       correspondingPlane[i][ipId][inId].reshape(1, 3),
                                       np.around(dist[j][0], 3).reshape(1, 1)))
            else:
                new = np.row_stack((new, np.column_stack((correspondingPlane[i][opId][onId].reshape(1, 3),
                                                          correspondingPlane[i][ipId][inId].reshape(1, 3),
                                                          np.around(dist[j][0], 3).reshape(1, 1)))))
            newId += 1
    distPcd = o3d.geometry.PointCloud()
    distPcd.points = o3d.utility.Vector3dVector(new[:, :6].reshape(-1, 3))
    distPcd.rotate(np.linalg.inv(rotationMatrix), center=center)
    new[:, :6] = np.asarray(distPcd.points).reshape(-1, 6)
    outputPathDist = os.path.join(args.outputPath, args.fileName.split('.')[0] + '_dist.txt')
    np.savetxt(outputPathDist, new)
    print('开间进深计算结果已保存')
    # 垂直度
    floorList = planeListFilter[floorId]  #找出地板的编号
    ceilingList = planeListFilter[ceilingId]#找出天花板编号
    pts, result = ceilingDeviationByFivePoints(ceilingList, args.ceilingDistToBoundary, args.gridSize)#Deviation偏差
    ceilingPcd = o3d.geometry.PointCloud()
    ceilingPcd.points = o3d.utility.Vector3dVector(pts.reshape(-1, 3))
    ceilingPcd.rotate(np.linalg.inv(rotationMatrix), center=center)
    transPts = np.asarray(ceilingPcd.points)
    ceilingResult = np.column_stack((transPts, result.reshape(-1, 1)))
    outputPathResult = os.path.join(args.outputPath, args.fileName.split('.')[0] + '_Result.txt')

    print('顶板极差计算结果为{0}'.format(result[-1]))
    wallId = [i for i in range(len(planeListFilter)) if i not in horzontalPlaneId] #找出墙的id
    height = np.min(ceilingList[:, 2]) - np.max(floorList[:, 2])
    ##################
    # 表面平整度和垂直度
    threads = []
    for i, plane in enumerate(planeListFilter):
        p = multiprocessing.Process(target=flatnessCal, args=(
        i, candidateList[i], plane, wallId, ceilingId, boundaryList[i], referencePoint, correspondingList, height, rotationMatrix,
        center, args)) #多线程计算平整度
        p.start()
        threads.append(p)
    for i in range(len(planeListFilter)):
        threads[i].join()
        print('第{0}个进程已结束'.format(i))

    fileList = os.listdir(args.outputPath)
    #################
    #################
    #################
    #################
    #################
    with open(outputPathResult, mode='w', encoding='utf-8') as f:
        for file in fileList:
            print(file)
            if file.endswith('pcd'):
                objectName = file.split('_')[1]
                # 文件名字不同所以 序列号也不一样
                pcd = o3d.io.read_point_cloud(os.path.join(args.outputPath, file))
                points = np.asarray(pcd.points)
                normals = np.asarray(pcd.normals)
                flatness = normals[:, 1]
                idx = np.where(flatness != 0)[0]
                selectedId = np.random.randint(0, idx.shape[0], 5)
                point = points[idx[selectedId]]
                flatnessOutput = np.column_stack((point, flatness[idx[selectedId]].reshape(-1, 1)))
                verticalityOutput = np.column_stack((point, normals[idx[selectedId], 2].reshape(-1, 1)))
                if objectName == 'ceiling':
                    f.write('{0} X-Y-Z-顶板极差(mm)\n'.format(file.split('.')[0]))
                    for i in range(5):
                        f.writelines(''.join(str(x)+' ' for x in ceilingResult[i]))
                        f.write('\n')
                    f.write('\n')
                    f.write('{0} X-Y-Z-平整度(mm)\n'.format(file.split('.')[0]))
                    for i in range(5):
                        f.writelines(''.join(str(x)+' ' for x in flatnessOutput[i]))
                        f.write('\n')
                    f.write('\n')
                if objectName == 'floor':
                    f.write('{0} X-Y-Z-平整度(mm)\n'.format(file.split('.')[0]))
                    for i in range(5):
                        f.writelines(''.join(str(x)+' ' for x in flatnessOutput[i]))
                        f.write('\n')
                    f.write('\n')
                if objectName == 'wall':
                    f.write('{0} X-Y-Z-垂直度(mm)\n'.format(file.split('.')[0]))
                    for i in range(5):
                        f.writelines(''.join(str(x)+' ' for x in verticalityOutput[i]))
                        f.write('\n')
                    f.write('\n')
                    f.write('{0} X-Y-Z-平整度(mm)\n'.format(file.split('.')[0]))
                    for i in range(5):
                        f.writelines(''.join(str(x)+' ' for x in flatnessOutput[i]))
                        f.write('\n')
                    f.write('\n')
    print('平整度与垂直度结果已保存')
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

if __name__ == '__main__':
    # args = parse_args()
    # start = time.time()
    # main(args)
    # end = time.time()
    # print('计算总时长：{0}'.format(end - start))
    path = r'/data/final_预处理步骤/indoor_data_mini/Room-1-Bathroom/Room-1-Bathroom.txt'
    point = dataOpen(path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point[:,:3])
    a, R, b, c = dataClean(pcd)
    print(R)
    np.savetxt('1.txt', np.array(a.points))