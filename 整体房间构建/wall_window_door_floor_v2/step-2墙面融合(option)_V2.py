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

def wall_data_read(path):
    "读取墙体数据: Input : wall_data_path   Ouput: wall_data_list(列表格式)"
    wall_data_paths = os.listdir(path)
    wall_data = []
    for wall_data_path in wall_data_paths:
        temp = os.path.join(path,wall_data_path)
        wall_data.append(dataOpen(temp))
    return wall_data, wall_data_paths
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
def dataClean(data, distThres=0.2):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
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

    # 用dbscan去除太远的离散数据
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            slicePcd.cluster_dbscan(eps=distThres, min_points=5, print_progress=True)).tolist()                       # 房屋0.2m

    maxlabel = max(labels, key=labels.count)
    ind = np.where(np.array(labels) == maxlabel)[0]                                                                   # 选择点数量最大的类别
    slicePcd = slicePcd.select_by_index(ind)
    aabb = slicePcd.get_axis_aligned_bounding_box()                                                                   # 获取切片数据的轴对齐包围框
    aabbPoints = np.asarray(aabb.get_box_points())
    aabb = o3d.geometry.PointCloud()
    aabb.points = o3d.utility.Vector3dVector(dataExpend(aabbPoints))                                                  # 将点云在z轴方向扩展并提取数据点
    aabb = aabb.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    finalpcd= pcd.crop(aabb)
    return np.asarray(finalpcd.points)
def wall_thickness_height(wall):
    wall_max = np.max(wall, axis=0)
    wall_min = np.min(wall, axis=0)
    temp = abs(wall_max - wall_min)
    thickness_temp = min(temp[0], temp[1])
    height_temp = temp[2]
    return thickness_temp, height_temp

def plane_ransac(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                             ransac_n=10,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = pcd.select_by_index(inliers)
    aabb = inlier_cloud.get_axis_aligned_bounding_box()
    res = aabb.get_center()
    return np.asarray([a,b,c]), res, np.asarray([a,b,c,d])
def wall_thickness_height(wall):
    wall_max = np.max(wall, axis=0)
    wall_min = np.min(wall, axis=0)
    temp = abs(wall_max - wall_min)
    thickness_temp = min(temp[0], temp[1])
    height_temp = temp[2]
    return thickness_temp, height_temp

def uniform_sampling(point_cloud, expand_time):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
    uni_down_pcd = pcd.uniform_down_sample(every_k_points=int(expand_time))
    return np.asarray(uni_down_pcd.points)

def point_cloud_plane_project(points, coefficients):
    """
    点云投影到平面
    :param points:输入点云
    :param coefficients: 待投影的平面
    :return: 投影后的点云
    """
    # 获取平面系数
    A = coefficients[0]
    B = coefficients[1]
    C = coefficients[2]
    D = coefficients[3]
    # 构建投影函数
    Xcoff = np.array([B * B + C * C, -A * B, -A * C])
    Ycoff = np.array([-B * A, A * A + C * C, -B * C])
    Zcoff = np.array([-A * C, -B * C, A * A + B * B])
    # 三维坐标执行投影
    xp = np.dot(points, Xcoff) - A * D
    yp = np.dot(points, Ycoff) - B * D
    zp = np.dot(points, Zcoff) - C * D
    project_points = np.c_[xp, yp, zp]  # 投影后的三维坐标
    project_sample_points = uniform_sampling(project_points)
    return np.round(project_sample_points, 1)

def euclidean_distance(p1, p2):
    return np.sqrt(np.sum((p1 - p2)**2))

def near_distance_points(points,wall_data_normals):
    points = np.array(points)
    # 初始化最近点和第二近点的索引列表
    nearest_indices = []
    for i in range(len(points)):
        #找到轴
        axis = np.argmax(abs(wall_data_normals[i]))
        # 计算当前点到所有其他点的距离
        if axis > 1:
            distances = np.linalg.norm(points - points[i], axis=1)
        else:
            distances = abs(points[:, axis] - points[i, axis])
        # 将当前点的距离设置为无穷大，以便不会选到自己
        distances[i] = np.inf
        # 找到最近和第二近的点的索引
        nearest_index = np.argmin(distances)
        distances[nearest_index] = np.inf  # 设置最近点的距离为无穷大
        second_nearest_index = np.argmin(distances)
        distances[second_nearest_index] = np.inf  # 设置最近点的距离为无穷大
        third_nearest_index = np.argmin(distances)
        # 记录索引
        nearest_indices.append([nearest_index, second_nearest_index, third_nearest_index])
    return nearest_indices
def wall_rh_id(wall_data, wall_data_normals, wall_reps, wall_names,threshold = 0.5):
    wall_len = len(wall_reps)
    rh = []
    nearest_reps = near_distance_points(wall_reps, wall_data_normals)
    for wall_id1 in range(wall_len):
        for wall_id2 in range(wall_id1+1, wall_len):
            if wall_id2 not in nearest_reps[wall_id1]:
                continue
            wall_rep1, wall_rep2 = wall_reps[wall_id1], wall_reps[wall_id2]
            normal1, normal2 = wall_data_normals[wall_id1], wall_data_normals[wall_id2]
            wall_max1, wall_max2 = np.max(wall_data[wall_id1], axis=0), np.max(wall_data[wall_id2], axis=0)
            wall_min1, wall_min2 = np.min(wall_data[wall_id1], axis=0), np.min(wall_data[wall_id2], axis=0)
            axis1, axis2 = np.argmax(abs(normal1)), np.argmax(abs(normal2))
            if axis1 == axis2 :#法向量最大值相同
                dist_axis = abs(wall_rep1[axis1] - wall_rep2[axis1])
                dist_flag = (wall_max1[1-axis1]-0.2) > wall_min2[1-axis1] and (wall_max2[1-axis1]-0.2) > wall_min1[1-axis1]
                if (0.1 < dist_axis < threshold) and dist_flag:  #曼哈顿距离
                    print(str(wall_names[wall_id1])+'与'+str(wall_names[wall_id2])+'进行墙体融合')
                    #######x轴扩长墙 y轴扩短墙#########
                    # 判断长短墙
                    if abs(wall_max1[1-axis1] - wall_min1[1-axis1]) > abs(wall_max2[1-axis1] - wall_min2[1-axis1]):
                        short_wall, long_wall = wall_id2, wall_id1  #
                    else:
                        short_wall, long_wall = wall_id1, wall_id2
                    # 针对不同axis进行墙体的rh
                    rh.append([long_wall, short_wall, axis1])# 一个是要扩充的墙 第二个是要被rh的墙 dis表示两者之间距离 axis表示在哪一个坐标轴工作 direction表示方向
    return rh
def generate_uniform_point_cloud(x_range, y_range, z_range, num_points=2*100000):
    """
    Generate a uniform 3D point cloud within specified x, y, z ranges.

    Parameters:
    x_range (tuple): The (min, max) range for the x coordinates.
    y_range (tuple): The (min, max) range for the y coordinates.
    z_range (tuple): The (min, max) range for the z coordinates.
    num_points (int): The total number of points to generate.

    Returns:
    np.ndarray: An array of shape (num_points, 3) containing the generated 3D points.
    """
    # Calculate the number of points along each dimension
    num_points_per_dim = int(np.ceil(num_points ** (1 / 3)))

    # Generate evenly spaced points along each dimension
    x = np.linspace(x_range[0], x_range[1], num_points_per_dim)
    y = np.linspace(y_range[0], y_range[1], num_points_per_dim)
    z = np.linspace(z_range[0], z_range[1], num_points_per_dim)

    # Create a 3D grid of points
    X, Y, Z = np.meshgrid(x, y, z)

    # Flatten the grid to create a point cloud
    points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

    # If there are too many points, randomly sample to get the exact number needed
    if points.shape[0] > num_points:
        indices = np.random.choice(points.shape[0], num_points, replace=False)
        points = points[indices]

    return points

def wall_expand(wall_data1, wall_data2, axis):
    wall1_wall2_rh_temp = uniform_sampling(np.vstack((wall_data1, wall_data2)), 2)
    z_min, z_max = np.min(wall_data1[:, 2]), np.max(wall_data1[:, 2])
    rate = 0
    if axis == 0:
        x_min, x_max = np.min(wall1_wall2_rh_temp[:, 0]) + rate * 0.2, np.max(wall1_wall2_rh_temp[:, 0]) - rate * 0.2
        y_min, y_max = min(np.min(wall_data1[:, 1]), np.min(wall_data2[:, 1])), max(np.max(wall_data1[:, 1]), np.max(wall_data2[:, 1]))
        final_thick = x_max - x_min
    else:
        y_min, y_max = np.min(wall1_wall2_rh_temp[:, 1]) + rate * 0.2, np.max(wall1_wall2_rh_temp[:, 1]) - rate * 0.2
        x_min, x_max = min(np.min(wall_data1[:, 0]), np.min(wall_data2[:, 0])), max(np.max(wall_data1[:, 0]), np.max(wall_data2[:, 0]))
        final_thick = y_max - y_min
    final_wall_data = generate_uniform_point_cloud([x_min,x_max], [y_min, y_max], [z_min, z_max])
    return final_wall_data, final_thick
def wall_rh_3D(wall_datas, rh_list):
    #扩充数据
    wall_datas_del_id = []
    for rh in rh_list:
        wall_expand_id, wall_not_expand_id, axis = rh[0], rh[1], rh[2],
        wall_datas_del_id.append(wall_not_expand_id)
        wall_data1, wall_data2 = wall_datas[wall_expand_id], wall_datas[wall_not_expand_id]
        temp_data, temp_thick = wall_expand(wall_data1, wall_data2, axis)
        wall_datas[wall_expand_id] = temp_data
    return wall_datas, wall_datas_del_id

def aabb_(wall_data, vis_flag = False):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(wall_data[:,:3])
    aabb = pcd.get_axis_aligned_bounding_box()
    vertex_set = np.asarray(aabb.get_box_points())
    vertex_set = np.round(vertex_set, 6)
    print("aabb包围盒的顶点为：\n", vertex_set)
    bound = aabb.get_max_bound() - aabb.get_min_bound()
    print(bound)
    wall_thickness, wall_height = np.asarray(min(bound)), np.asarray(bound[2])
    if wall_thickness != bound[0]:
        wall_length = bound[1]
    else:
        wall_length = bound[2]
    print("aabb包围盒边长的高度为：\n", wall_height)
    print("aabb包围盒边长的厚度为：\n", wall_thickness)
    print("aabb包围盒边长的长度为：\n", wall_length)

    if vis_flag:
        aabb.color = (1, 0, 0)  # aabb包围盒为红色
        o3d.visualization.draw_geometries([pcd, aabb], window_name="AABB包围盒",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    return wall_thickness, wall_height, wall_length, vertex_set
def wall_2d_plot(wall_reps):
    x, y = [], []
    for i in wall_reps:
        x.append(i[0]), y.append(i[1])
    # 画图
    plt.scatter(x, y,  edgecolors='r')
    plt.show()

def vis_from_list(list):
    data = np.zeros((1,3))
    for i in list:
        data = np.vstack((data,i))
    data = np.delete(data, 0, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])

    pcd.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([pcd], window_name="ctz",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

def main():
    start = time.time()
    input_path = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐\wall'
    points_save_path = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐\wall_rh'
    if not os.path.exists(points_save_path):
        os.makedirs(points_save_path)  # 如果不存在，新建文件夹
    wall_datas, wall_name = wall_data_read(input_path)

    #**************************************************数据预处理部分**************************************************#
    #计算墙体的 法向量 高度 以及二维墙代表xy点
    wall_data_clean, wall_data_normals,wall_reps=[], [], []
    plane_list, all_h_temp = [], 0
    for wall_data in wall_datas:
        temp0 = dataClean(wall_data)                                       #清洗数据的同时
        normal, wall_rep, plane = plane_ransac(temp0)                     #进行墙面融合进行准备
        wall_data_clean.append(temp0), wall_data_normals.append(normal)
        wall_reps.append([wall_rep[0], wall_rep[1]]), plane_list.append(plane)

    #**************************************************墙壁融合主要代码**************************************************#
    # wall_2d_plot(wall_reps)    #可视化2d墙体空间分布
    rh_list = wall_rh_id(wall_data_clean, wall_data_normals,wall_reps, wall_name)
    wall_data_clean_final_temp, wall_datas_del_id = wall_rh_3D(wall_data_clean, rh_list)
    end = time.time()
    print('计算总时长：{0}'.format(end - start))

    # #**************************************************将结果写入其他文件**************************************************#
    wall_data_clean_final = [x for i, x in enumerate(wall_data_clean_final_temp) if i not in wall_datas_del_id]
    wall_data_clean_final_name = [x for i, x in enumerate(wall_name) if i not in wall_datas_del_id]
    for i in range(len(wall_data_clean_final)):
        np.savetxt(os.path.join(points_save_path, wall_data_clean_final_name[i]), wall_data_clean_final[i])


if __name__ == '__main__':
    main()

