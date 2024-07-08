import open3d as o3d
import numpy as np
import os
from sklearn.neighbors import KDTree
from sklearn.neighbors import LocalOutlierFactor
import matplotlib.pyplot as plt
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
    return data
#/******************************************************尺寸估计******************************************************/
def vis2d(project_data, data_vertex_set_slices):
    project_data, data_vertex_set_slices = np.array(project_data), np.array(data_vertex_set_slices)
    plt.scatter(project_data[:, 0], project_data[:, 1], c='blue', label='Other Points')
    plt.scatter(data_vertex_set_slices[:, 0], data_vertex_set_slices[:, 1], c='red', marker='x', s=100,
                label='Target Point')

    plt.legend()
    plt.title("2D Points and the Closest Point to the Target Point")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()
def dis_outliers_index(distances):
    distances_sort = sorted(distances, reverse=True)
    if distances_sort[0] > 0.2:
        return distances.index(max(distances))
    else:
        return -1
def corner_refinement(data, data_vertex_set, vis_flag = False):
    zmin, zmax = np.min(data_vertex_set, axis=0)[2], np.max(data_vertex_set, axis=0)[2]
    ymin, ymax = np.min(data_vertex_set, axis=0)[1], np.max(data_vertex_set, axis=0)[1]
    xmin, xmax = np.min(data_vertex_set, axis=0)[0], np.max(data_vertex_set, axis=0)[0]

    data_vertex_set_slices = []
    project_data = np.array(data[:,:2]).reshape(-1, 2)
    for i in data_vertex_set:
        if i[2] == zmin:
            data_vertex_set_slices.append([i[0], i[1]])
    tree, min_distances = KDTree(project_data), []
    #可视化
    if vis_flag:
        vis2d(project_data, data_vertex_set_slices)

    for corner_point in data_vertex_set_slices:
        dist, ind = tree.query(np.array(corner_point).reshape(-1, 2), k=1)
        min_distance = dist[0][0]
        min_distances.append(min_distance)
    index = dis_outliers_index(min_distances)
    if index == -1:
        return data_vertex_set
    else:
        # print('需要分裂')
        #/****************需要进行角点分裂 假定是1分裂成3*****************/
        temp, step = data_vertex_set_slices[index], 0.1
        #/****************先进行x轴分裂*****************/
        temp1 = [0, temp[1]]
        for i in range(1,int((xmax - xmin)/step)):
            if temp[0] == xmin:
                temp1[0] = xmin + i*step
            else:
                temp1[0] = xmax - i*step
            dist_temp, _ = tree.query(np.array(temp1).reshape(-1, 2), k=1)
            if dist_temp < min(min_distances) + 0.1:
                break
        #/****************先进行y轴分裂*****************/
        temp2 = [temp[0], 0]
        for i in range(1,int((ymax - ymin)/step)):
            if temp[1] == ymin:
                temp2[1] = ymin + i*step
            else:
                temp2[1] = ymax - i*step
            dist_temp, _ = tree.query(np.array(temp2).reshape(-1, 2), k=1)
            if dist_temp < min(min_distances) + 0.1:
                break
        # /****************先进行y轴分裂*****************/
        temp3 = [temp1[0], temp2[1]]
        data_vertex_set_slices.remove(data_vertex_set_slices[index])
        data_vertex_set_slices.extend([temp1,temp2,temp3])
        #可视化
        if vis_flag:
            vis2d(project_data, data_vertex_set_slices)
        # 写入角点信息
        data_vertex_set_slices = sorted(data_vertex_set_slices, key=lambda x: (x[0], x[1]))
        final_corner = []
        for corner_point in data_vertex_set_slices:
            temp1 = [corner_point[0], corner_point[1], zmin]
            temp2 = [corner_point[0], corner_point[1], zmax]
            final_corner.extend([temp1, temp2])
        final_corner = np.array([np.array([round(element, 6) for element in sublist]) for sublist in final_corner])
        return final_corner
def wall_se_aabb(wall_data, vis_flag = False):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(wall_data[:,:3])
    aabb = pcd.get_axis_aligned_bounding_box()
    vertex_set = np.asarray(aabb.get_box_points())
    vertex_set = np.round(vertex_set, 6)
    # print("aabb包围盒的顶点为：\n", vertex_set)
    bound = aabb.get_max_bound() - aabb.get_min_bound()
    # print(bound)
    wall_thickness, wall_height = np.asarray(min(bound)), np.asarray(bound[2])
    if wall_thickness != bound[0]:
        wall_length = bound[1]
    else:
        wall_length = bound[2]
    # print("aabb包围盒边长的高度为：\n", wall_height)
    # print("aabb包围盒边长的厚度为：\n", wall_thickness)
    # print("aabb包围盒边长的长度为：\n", wall_length)

    if vis_flag:
        aabb.color = (1, 0, 0)  # aabb包围盒为红色
        o3d.visualization.draw_geometries([pcd, aabb], window_name="AABB包围盒",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    return wall_thickness, wall_height, wall_length, vertex_set
def data_dir_read(path):
    "读取墙体数据: Input : wall_data_path   Ouput: wall_data_list(列表格式)"
    wall_data_paths = os.listdir(path)
    wall_data = []
    for wall_data_path in wall_data_paths:
        temp = os.path.join(path,wall_data_path)
        wall_data.append(dataOpen(temp))
    return wall_data, wall_data_paths

def point_vis(points_path, vis_flag=True):
    points = dataOpen(points_path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6])
    if vis_flag:
        o3d.visualization.draw_geometries([pcd], window_name="ctz",
                                          width=1024, height=768,
                                          left=50, top=50,
                                          mesh_show_back_face=False)
def main():
    #对齐后的文件夹数据
    input_data_files = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\data_对齐_test'
    output_information_files = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\wall_window_door_floor_v2\element_information'

    for classes in os.listdir(input_data_files):
        # if classes == 'wall': continue
        # if classes.startswith('wall'):classes = 'wall'
        classes_count = 0
        input_class_data_path = os.path.join(input_data_files, classes)
        output_class_information_path = os.path.join(output_information_files, classes)
        if not os.path.isdir(output_class_information_path):
            os.mkdir(output_class_information_path)
        all_classes_data, _ = data_dir_read(input_class_data_path)
        for classes_data in all_classes_data:
            element_name = classes + '_' + str(classes_count)
            classes_count = classes_count + 1
            element_thickness, element_height, element_length, vertex_set = wall_se_aabb(classes_data)
            if classes == 'floor': vertex_set = corner_refinement(classes_data, vertex_set)
            with open(output_class_information_path + '\\' + element_name + '.txt', 'w', encoding="utf-8") as f:
                f.write('element名称：' + '\n' + element_name + '\n')
                f.write('element高度：' + '\n' + str(element_height) + '\n')
                f.write('element长度：' + '\n' + str(element_length) + '\n')
                f.write('element厚度：' + '\n' + str(element_thickness) + '\n')
                f.write('element包围框顶点：' + '\n')
                for i in range(len(vertex_set)):
                    f.write(str(vertex_set[i]) + '\n')



if __name__ == '__main__':
    main()






