import math
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import random
from scipy.optimize import fsolve
import pyransac3d as pyrsc


def are_vectors_aligned(v1, v2):
    '''
    计算两个向量是否平行，返回一个偏差
    :param v1:
    :param v2:
    :return:
    '''
    # 将向量转换为NumPy数组
    v1 = np.array(v1)
    v2 = np.array(v2)
    # 计算单位向量
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    # 计算点积
    dot_product = np.dot(unit_v1, unit_v2)
    m = abs(abs(dot_product)-1)
    return m

def rotate_point_cloud(pcd, source_vector, target_vector):
    '''
    旋转矩阵，当前向量和目标向量
    :param pcd:
    :param source_vector:
    :param target_vector:
    :return:
    '''
    # 将向量归一化
    source_vector = np.array(source_vector)
    target_vector = np.array(target_vector)
    source_vector_normalized = source_vector / np.linalg.norm(source_vector)
    target_vector_normalized = target_vector / np.linalg.norm(target_vector)
    # 计算旋转轴和旋转角度
    axis = np.cross(source_vector_normalized, target_vector_normalized)
    angle = np.arccos(np.dot(source_vector_normalized, target_vector_normalized))
    # 生成旋转矩阵
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
    # 应用旋转
    pcd.rotate(R, center=(0, 0, 0))
    # # 可选：保存旋转后的点云
    # output_file_name = "rotated_" + file_name
    # o3d.io.write_point_cloud(output_file_name, pcd)
    # print(f"Rotated point cloud saved as {output_file_name}")
    return pcd

def extract_planes(pcd):
    '''
    提取出一个原始数据我们想要的平面，对于规整的，我们
    可以认为首先提取的是一些比较大的平面，然后我们根据
    自己的参数法向量，就可以得到我们想要的平面。
    :param pcd:
    :return:
    '''
    output_dir = "ransac_fenge"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    planes = []
    ransac_n = 3
    while True:
        # 检查点云中的点数是否足够
        if len(pcd.points) < ransac_n:
            break
        # RANSAC平面分割
        distance_threshold = 0.02  # 内点到平面模型的最大距离
        num_iterations = 1000  # 最大迭代次数
        plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
        # if len(inliers) < 10000:  # 如果内点数量小于百分比，舍弃该平面
        #     break
        # 输出平面方程
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.5f}x + {b:.5f}y + {c:.5f}z + {d:.5f} = 0")
        # 平面内点点云
        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud = rotate_point_cloud(inlier_cloud, [a,b,c], [1,0,0])
        # 剩余点云数量小于拟合平面的点数，舍弃剩余点云
        if len(pcd.points) < 10000:
            break
        if are_vectors_aligned([a,b,c],[1,0,0]) <0.0001:
        # 添加平面到列表中
            planes.append(inlier_cloud)
        if len(planes) == 2:
            for i in range(len(planes)):
                slice_file_name = f"{output_dir}/ransac{i}.ply"
                o3d.io.write_point_cloud(slice_file_name, planes[i])
            break
        # 从点云中移除平面内点
        pcd = pcd.select_by_index(inliers, invert=True)
    return planes[0], planes[1]

def tongjilvbo(pcd):
    num_neighbors = 20  # K邻域点的个数
    std_ratio = 4  # 标准差乘数
    sor_pcd, ind = pcd.remove_statistical_outlier(num_neighbors, std_ratio)
    # sor_pcd.paint_uniform_color([0, 0, 1])
    # # print("统计滤波后的点云：", sor_pcd)
    # sor_pcd.paint_uniform_color([0, 0, 1])
    sor_noise_pcd = pcd.select_by_index(ind, invert=True)
    # print("噪声点云：", sor_noise_pcd)
    sor_noise_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([sor_pcd])
    return sor_pcd

def project_to_plane(pcd,n):
    points = np.asarray(pcd.points)
    # 将所有点的 z 坐标设置为 0
    points[:, n] = 0
    pcd.points = o3d.utility.Vector3dVector(points)
    # 可选：保存投影后的点云
    # o3d.visualization.draw_geometries([pcd])
    return pcd

def angle_between_vectors(v1, v2):
    x1, y1 = v1
    x2, y2 = v2
    # 计算点积和叉积
    dot_product = x1 * x2 + y1 * y2
    cross_product = x1 * y2 - y1 * x2
    # 使用 atan2 计算向量 v1 到 v2 的有向角度
    angle = math.atan2(cross_product, dot_product)
    # 将结果转换为度
    angle_degrees = math.degrees(angle)
    # 角度调整为最短路径的角度，并带有方向性
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360
    return angle_degrees


def convert_to_2d(pcd):
    # 将点云数据转换为numpy数组，并确保数据类型为 float64
    points = np.asarray(pcd.points, dtype=np.float64)
    # 检测每个维度是否为0
    zero_dim = np.all(points == 0, axis=0)  # 返回一个布尔数组，表示每个维度是否全部为0
    # 确定非零的两个维度
    non_zero_dims = np.where(~zero_dim)[0]
    if len(non_zero_dims) != 2:
        raise ValueError("The point cloud does not have exactly one zero dimension across all points.")
    # 提取两个非零维度
    new_points = points[:, non_zero_dims]
    # 为了符合Open3D的期望，添加一个全0的Z维度
    zeros = np.zeros((new_points.shape[0], 1), dtype=np.float64)
    new_points_3d = np.hstack((new_points, zeros))
    # 创建新的二维点云
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(new_points_3d)
    return new_pcd

def process_point_cloud(pcd):
    # 创建KDTree
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    # 准备颜色数组
    colors = np.zeros((np.asarray(pcd.points).shape[0], 3))  # 默认为黑色
    colors[:, 0] = 1  # 设置所有点为红色
    cemian_line_points = []  # 用于存储蓝色点的列表
    # 遍历每一个点
    for i in range(len(pcd.points)):
        # 当前点
        point = np.asarray(pcd.points)[i]
        # 在KDTree中搜索邻域内的100个点
        [k, idx, _] = pcd_tree.search_knn_vector_3d(point, 101)
        if k < 101:
            continue
        # 提取邻域点
        vectors = []
        for ind in idx[1:]:  # 排除自身
            neighbor_point = np.asarray(pcd.points)[ind]
            vector = (neighbor_point - point)[:2]  # 只考虑X和Y分量
            vectors.append(vector)
        if not vectors:
            continue
        # 随机选取一个向量作为参考向量
        ref_vector = random.choice(vectors)
        angles = []
        # 计算与参考向量之间的有向角度
        for vec in vectors:
            if np.array_equal(vec, ref_vector):
                continue  # 跳过与参考向量自身的比较
            if np.linalg.norm(vec) == 0 or np.linalg.norm(ref_vector) == 0:
                continue
            angle = angle_between_vectors(ref_vector, vec)
            angles.append(angle)
        if angles:
            max_angle = max(angles)
            min_angle = min(angles)
            # 判断是否为边界点
            if abs(180 - (max_angle - min_angle)) < 5 or abs(90 - (max_angle - min_angle)) < 5:
                colors[i] = [0, 0, 1]  # 设置为蓝色
                cemian_line_points.append(point)
        if i%100 == 0:
            print(i, max_angle, min_angle)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    # 将蓝色点存储为PLY文件
    if cemian_line_points:
        cemian_line_pcd = o3d.geometry.PointCloud()
        cemian_line_pcd.points = o3d.utility.Vector3dVector(np.array(cemian_line_points))
        # o3d.io.write_point_cloud("cemian_line_2.ply", cemian_line_pcd)
    o3d.visualization.draw_geometries([pcd])
    return cemian_line_pcd

def ransac_line(pcd):
    segment = []  # 存储分割结果的容器
    line_k = []
    line_b = []
    min_num = 15  # 每个分割直线所需的最小点数
    dist = 0.004  # Ransac分割的距离阈值
    iters = 0  # 用于统计迭代次数，非待设置参数
    # -----------------------------------分割多个直线-------------------------------------
    while iters < 4:
        points = np.asarray(pcd.points)
        line = pyrsc.Line()
        A, B, inliers = line.fit(points, thresh=dist, maxIteration=100)
        line_k.append(list(A))
        line_b.append(list(B))
        line_cloud = pcd.select_by_index(inliers)  # 分割出的直线点云
        r_color = [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[0,0,0],[0,1,1]]
        line_cloud.paint_uniform_color(r_color[iters])
        pcd = pcd.select_by_index(inliers, invert=True)  # 剩余的点云
        segment.append(line_cloud)
        # file_name = "RansacFitMutiLine" + str(iters + 1) + ".ply"
        # o3d.io.write_point_cloud(file_name, line_cloud)
        iters += 1
        if len(inliers) < min_num:
            break
    # ------------------------------------结果可视化--------------------------------------
    # o3d.visualization.draw_geometries(segment)
    # o3d.io.write_point_cloud("ransac_line.ply", [segment])
    return line_k, line_b

def find_intersection(direction1, point1, direction2, point2):
    # 定义两条直线的参数方程
    def line1(t):
        return point1 + direction1 * t[0]
    def line2(t):
        return point2 + direction2 * t[1]
    # 定义方程组，计算交点
    def equations(t):
        x1, y1, z1 = line1(t)
        x2, y2, z2 = line2(t)
        return (x1 - x2, y1 - y2, z1 - z2)

    direction1 = np.array(direction1)
    direction2 = np.array(direction2)
    point1 = np.array(point1)
    point2 = np.array(point2)
    # 初始猜测值
    t_initial = np.array([0.0, 0.0, 0.0])
    # 解方程组
    t_solution = fsolve(equations, t_initial)
    # 计算交点
    intersection_point = line1(t_solution)
    return intersection_point

def calculate_distance(point1, point2):
    # 将点坐标转换为 numpy 数组
    point1 = np.array(point1)
    point2 = np.array(point2)

    # 计算两点之间的距离
    distance = np.linalg.norm(point1 - point2)
    return distance

def categorize_and_average(points):
    # 将点坐标数组转换为NumPy数组
    points = np.array(points)
    # 计算x和y的中间值，用于分辨点的位置
    mid_x = np.mean(points[:, 1])
    mid_y = np.mean(points[:, 2])
    # 初始化四个角落的点数组
    top_left = []
    top_right = []
    bottom_left = []
    bottom_right = []
    # 分类点到四个角
    for point in points:
        if point[1] <= mid_x and point[2] >= mid_y:
            top_left.append(point)
        elif point[1] <= mid_x and point[2] < mid_y:
            bottom_left.append(point)
        elif point[1] > mid_x and point[2] >= mid_y:
            top_right.append(point)
        elif point[1] > mid_x and point[2] < mid_y:
            bottom_right.append(point)
    # 计算每个角落的平均坐标
    top_left_avg = np.mean(top_left, axis=0)
    top_right_avg = np.mean(top_right, axis=0)
    bottom_left_avg = np.mean(bottom_left, axis=0)
    bottom_right_avg = np.mean(bottom_right, axis=0)
    # 计算尺寸
    average_height = (
                (np.abs(top_left_avg[2] - bottom_left_avg[2]) + np.abs(top_right_avg[2] - bottom_right_avg[2])) / 2)
    average_width = (
                (np.abs(top_left_avg[1] - top_right_avg[1]) + np.abs(bottom_left_avg[1] - bottom_right_avg[1])) / 2)
    average_depth = (
                (np.abs(top_left_avg[0] - top_right_avg[0]) + np.abs(bottom_left_avg[0] - bottom_right_avg[0])) / 2)
    return average_height, average_width, average_depth

def main():
    # 提取出两个侧面
    # print("->正在RANSAC平面分割...")
    # pcd = o3d.io.read_point_cloud("D:/maybe/nantong/successs/3jieduan/T8NY.ply")
    # # pcd = pcd.uniform_down_sample(1)  # 对点云进行均匀下采样，每10个点选取一个点
    # print(pcd)#看看当前点有多少点
    # plane_1, plane_2 = extract_planes(pcd)#提取两个侧边板，提取出我们想要的板平面（参数在函数里面），并进行保存。
    # o3d.visualization.draw_geometries([plane_1])
    # o3d.visualization.draw_geometries([plane_2])

    # 对侧面进行数据处理，最终得到一个平面z坐标为0
    # pcd = o3d.io.read_point_cloud("D:/maybe/nantong/successs/3jieduan/cemian/ransac_fenge/ransac0.ply")
    # pcd = tongjilvbo(pcd)
    # pcd = project_to_plane(pcd, 0)
    # print(pcd)
    # o3d.io.write_point_cloud("ban_1.ply",pcd)

    # 得到边界点，将边界点保存
    pcd = o3d.io.read_point_cloud("D:/maybe/nantong/successs/3jieduan/cemian/ban_1.ply")
    pcd = convert_to_2d(pcd)
    pcd = pcd.uniform_down_sample(100)  # 对点云进行均匀下采样，每10个点选取一个点
    print(pcd)
    # 处理点云
    processed_pcd = process_point_cloud(pcd)
    # o3d.io.write_point_cloud("ban_bianjie_1.ply",processed_pcd)

    # 开始进行边界直线拟合
    # pcd = o3d.io.read_point_cloud("ban_bianjie_1.ply")
    # print("开始拟合直线")
    # distance_H1_sum = 0
    # distance_L1_sum = 0
    # distance_H2_sum = 0
    # distance_L2_sum = 0
    # n = 20
    # for i in range(n):
    #     direction, point = ransac_line(pcd)
    #     # print(direction, point)
    #     # 红绿蓝黄黑青
    #     point1 = find_intersection(direction[0], point[0], direction[1], point[1])
    #     # print("角点1坐标为：", point1)
    #     point2 = find_intersection(direction[1], point[1], direction[3], point[3])
    #     # print("角点2坐标为：", point2)
    #     point3 = find_intersection(direction[2], point[2], direction[3], point[3])
    #     # print("角点4坐标为：", point3)
    #     point4 = find_intersection(direction[0], point[0], direction[2], point[2])
    #     # print("角点3坐标为：", point4)
    #
    #     distance_H1 = calculate_distance(point1, point2)
    #     # print("H1距离为：", distance_H1)
    #     distance_H1_sum = distance_H1_sum + distance_H1
    #
    #     distance_L1 = calculate_distance(point2, point3)
    #     # print("L1距离为：", distance_L1)
    #     distance_L1_sum = distance_L1_sum + distance_L1
    #
    #     distance_H2 = calculate_distance(point3, point4)
    #     # print("H2距离为：", distance_H2)
    #     distance_H2_sum = distance_H2_sum + distance_H2
    #
    #     distance_L2 = calculate_distance(point4, point1)
    #     # print("L2距离为：", distance_L2)
    #     distance_L2_sum = distance_L2_sum + distance_L2
    # print(f"板高度为：",1000*((distance_H1_sum + distance_H2_sum)/(2*n)),"mm")
    # print(f"板宽度为：",1000*((distance_L1_sum + distance_L2_sum)/(2*n)),"mm")





    # pcd = o3d.io.read_point_cloud("cemian_line_2.ply")
    # print("开始拟合直线")
    # distance_H1_sum = 0
    # distance_L1_sum = 0
    # distance_H2_sum = 0
    # distance_L2_sum = 0
    # n = 20
    # for i in range(n):
    #     direction, point = ransac_line(pcd)
    #     print(direction, point)
    #     # 红绿蓝黄黑青
    #     point1 = find_intersection(direction[0], point[0], direction[2], point[2])
    #     print("角点1坐标为：", point1)
    #     point2 = find_intersection(direction[2], point[2], direction[1], point[1])
    #     print("角点2坐标为：", point2)
    #     point3 = find_intersection(direction[1], point[1], direction[3], point[3])
    #     print("角点4坐标为：", point3)
    #     point4 = find_intersection(direction[0], point[0], direction[3], point[3])
    #     print("角点3坐标为：", point4)
    #
    #     distance_H1 = calculate_distance(point1, point2)
    #     print("H1距离为：", distance_H1)
    #     distance_H1_sum = distance_H1_sum + distance_H1
    #
    #     distance_L1 = calculate_distance(point2, point3)
    #     print("L1距离为：", distance_L1)
    #     distance_L1_sum = distance_L1_sum + distance_L1
    #
    #     distance_H2 = calculate_distance(point3, point4)
    #     print("H2距离为：", distance_H2)
    #     distance_H2_sum = distance_H2_sum + distance_H2
    #
    #     distance_L2 = calculate_distance(point4, point1)
    #     print("L2距离为：", distance_L2)
    #     distance_L2_sum = distance_L2_sum + distance_L2
    # print((distance_H1_sum + distance_H2_sum)/(2*n))
    # print((distance_L1_sum + distance_L2_sum)/(2*n))

if __name__ == "__main__":
    main()