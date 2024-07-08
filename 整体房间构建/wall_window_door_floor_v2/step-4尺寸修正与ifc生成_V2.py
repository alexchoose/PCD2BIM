import numpy as np
import os
import sys
from IfcOpenHouse.ios_utils import (
    IfcOpenShellPythonAPI, placement_matrix, clipping, ColourRGB, TerrainBuildMethod,
    build_native_bspline_terrain, build_tesselated_occ_terrain, ios_entity_overwrite_hook
)
from ifc_generation_V2 import create_ifc_wall, create_ifc_door, create_ifc_window, create_ifc_footing

def ifc_generation_based_on_info(element_information_ref, door_wall_names, window_wall_names, output_file):
    # Data definition
    project_name = 'IFC Open House'
    site_name, building_name, storey_name = 'OSArch Land', 'Open house', 'Ground floor'
    ios = IfcOpenShellPythonAPI()  # q1: thoughts about a data-scientish "import ifcopenshell.api as ios"?
    file = ios.project.create_file(version='IFC4')
    project = ios.root.create_entity(file, ifc_class='IfcProject', name=project_name)
    ios.project.assign_declaration(file, definition=project,
                                   relating_context=project)  # q2: from my ignorance, is this necessary?
    ios.unit.assign_unit(
        file, length={'is_metric': True, 'raw': 'METERS'}, area={'is_metric': True, 'raw': 'METERS'},
        volume={'is_metric': True, 'raw': 'METERS'}
    )
    ctx = ios.context.add_context(file, context_type='Model')
    body = ios.context.add_context(
        # q3: isn't this screaming for "context.add_subcontext"? also, context_type may be redundant
        file, context_type='Model', context_identifier='Body', target_view='MODEL_VIEW', parent=ctx
    )
    sys.addaudithook(
        ios_entity_overwrite_hook(file, sys.modules[__name__], do_not_delete=[project, ctx, body])
    )
    # /********************************定义整体房间与楼层********************************/
    site = ios.root.create_entity(file, ifc_class='IfcSite', name=site_name)
    ios.aggregate.assign_object(file, product=site, relating_object=project)
    building = ios.root.create_entity(file, ifc_class='IfcBuilding', name=building_name)
    ios.aggregate.assign_object(file, product=building, relating_object=site)
    storey = ios.root.create_entity(file, ifc_class='IfcBuildingStorey', name=storey_name)
    ios.aggregate.assign_object(file, product=storey, relating_object=building)
    # /********************************墙体定义********************************/
    walls_info, walls_ifc = element_information_ref['wall'], {}
    for wall_info in walls_info:
        wall_height, wall_thickness, wall_length = wall_info['height'], wall_info['thickness'], wall_info['length']
        wall_name, wall_vps = wall_info['name'], wall_info['element_vps']
        walls_ifc[wall_name] = create_ifc_wall(ios, file, storey, body, wall_vps, wall_height, wall_name)
    # /********************************door定义********************************/
    doors_info, doors_ifc = element_information_ref['door'], []
    for door_info in doors_info:
        door_height, door_thickness, door_length = door_info['height'], door_info['thickness'], door_info['length']
        door_name, door_vps = door_info['name'], door_info['element_vps']
        wall_related_ifc = walls_ifc[door_wall_names[door_info['name']]]
        create_ifc_door(ios, file, storey, body, door_vps, door_height, door_name, wall_related_ifc)
    # /********************************window定义********************************/
    windows_info, windows_ifc = element_information_ref['window'], []
    for window_info in windows_info:
        window_height, window_thickness, window_length = window_info['height'], window_info['thickness'], window_info['length']
        window_name, window_vps = window_info['name'], window_info['element_vps']
        wall_related_ifc = walls_ifc[window_wall_names[window_info['name']]]
        create_ifc_window(ios, file, storey, body, window_vps, window_height, window_name, wall_related_ifc)
    # /********************************floor定义********************************/
    floors_info, floors_ifc = element_information_ref['floor'], []
    for floor_info in floors_info:
        floor_height, floor_thickness, floor_length = floor_info['height'], floor_info['thickness'], floor_info['length']
        floor_name, floor_vps = floor_info['name'], floor_info['element_vps']
        create_ifc_footing(ios, file, storey, body, floor_vps, floor_height, floor_name)

    file.write(output_file)
def wall_refinement(wall_vps):
    def wall_thickness_refinement_gongmian(wall_vps, threshold=0.475):
        wall_x_range, wall_y_range = [], []
        wall_x_range_new, wall_y_range_new = [], []
        for wall_vp in wall_vps:
            wall_vp = np.array(wall_vp)
            temp0, temp1 = np.min(wall_vp, axis=0), np.max(wall_vp, axis=0)
            wall_x_range.append([temp0[0], temp1[0]]), wall_y_range.append([temp0[1], temp1[1]])
            wall_x_range_new.append([temp0[0], temp1[0]]), wall_y_range_new.append([temp0[1], temp1[1]])
        for id1 in range(len(wall_y_range_new)):
            wall_x_range1, wall_y_range1 = wall_x_range_new[id1], wall_y_range_new[id1]
            if (wall_x_range1[1] - wall_x_range1[0]) > (wall_y_range1[1] - wall_y_range1[0]):
                axis = 1
            else:
                axis = 0
            for id2 in range(id1 + 1, len(wall_y_range)):
                wall_x_range2, wall_y_range2 = wall_x_range_new[id2], wall_y_range_new[id2]
                if axis == 0:
                    if sum(abs(np.array(wall_x_range2) - np.array(wall_x_range1))) < threshold:
                        if wall_x_range1[0] > wall_x_range2[0]:
                            wall_x_range1[0] = wall_x_range2[0]
                        else:
                            wall_x_range2[0] = wall_x_range1[0]
                        if wall_x_range1[1] < wall_x_range2[1]:
                            wall_x_range1[1] = wall_x_range2[1]
                        else:
                            wall_x_range2[1] = wall_x_range1[1]
                        continue
                else:
                    if sum(abs(np.array(wall_y_range2) - np.array(wall_y_range1))) < threshold:
                        if wall_y_range1[0] > wall_y_range2[0]:
                            wall_y_range1[0] = wall_y_range2[0]
                        else:
                            wall_y_range2[0] = wall_y_range1[0]
                        if wall_y_range1[1] < wall_y_range2[1]:
                            wall_y_range1[1] = wall_y_range2[1]
                        else:
                            wall_y_range2[1] = wall_y_range1[1]
                        continue
        for id in range(len(wall_vps)):
            wall_vp = wall_vps[id]
            wall_x_range_id, wall_y_range_id = wall_x_range[id], wall_y_range[id]
            wall_x_range_id_new, wall_y_range_id_new = wall_x_range_new[id], wall_y_range_new[id]
            for point in wall_vp:
                if point[0] in wall_x_range_id:
                    index_x = wall_x_range_id.index(point[0])
                    point[0] = wall_x_range_id_new[index_x]
                if point[1] in wall_y_range_id:
                    index_y = wall_y_range_id.index(point[1])
                    point[1] = wall_y_range_id_new[index_y]
        return wall_vps

    def wall_thickness_refinement_chuizhi(wall_vps, threshold=0.5):  # (有三种不同状态改)
        wall_x_range, wall_y_range = [], []
        wall_x_range_new, wall_y_range_new = [], []
        for wall_vp in wall_vps:
            wall_vp = np.array(wall_vp)
            temp0, temp1 = np.min(wall_vp, axis=0), np.max(wall_vp, axis=0)
            wall_x_range.append([temp0[0], temp1[0]]), wall_y_range.append([temp0[1], temp1[1]])
            wall_x_range_new.append([temp0[0], temp1[0]]), wall_y_range_new.append([temp0[1], temp1[1]])
        for id1 in range(len(wall_y_range_new)):
            wall_x_range1, wall_y_range1 = wall_x_range_new[id1], wall_y_range_new[id1]
            for id2 in range(len(wall_y_range)):
                if id1 == id2:
                    continue
                wall_x_range2, wall_y_range2 = wall_x_range_new[id2], wall_y_range_new[id2]
                # 镶嵌状态
                if (wall_y_range1[1] - wall_y_range1[0] > threshold) & (
                        wall_x_range2[1] - wall_x_range2[0] > threshold):
                    # 镶嵌状态
                    if (wall_x_range1[1] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (
                            wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):  # 镶嵌1
                        wall_y_range1[1] = wall_y_range2[1]
                        continue
                    elif (wall_x_range2[1] > wall_x_range1[1]) & (wall_x_range1[0] > wall_x_range2[0]) & (
                            wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):  # 镶嵌2
                        wall_y_range1[1] = wall_y_range2[1]
                        continue
                    elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] >= wall_x_range1[0]) & (
                            wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):  # 镶嵌3
                        # wall_x_range2[0] = wall_x_range1[0]
                        wall_y_range1[1] = wall_y_range2[1]
                        continue
                        # 镶嵌状态
                    elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] > wall_x_range1[0]) & (
                            wall_y_range1[1] > wall_y_range2[1]) & (wall_y_range2[0] > wall_y_range1[0]):  # 镶嵌4
                        # wall_x_range2[0] = wall_x_range1[0]
                        continue
                    elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] > wall_x_range1[0]) & (
                            wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌5
                        wall_y_range1[0] = wall_y_range2[0]
                        continue
                    elif (wall_x_range2[1] > wall_x_range1[1]) & (wall_x_range1[0] > wall_x_range2[0]) & (
                            wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌6
                        wall_y_range1[0] = wall_y_range2[0]
                        continue
                    elif (wall_x_range1[1] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (
                            wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌7
                        wall_y_range1[0] = wall_y_range2[0]
                        # wall_x_range2[1] = wall_x_range1[1]
                        continue
                    elif (wall_x_range1[0] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (
                            wall_y_range1[1] > wall_y_range2[1]) & (wall_y_range2[0] > wall_y_range1[0]):  # 镶嵌8
                        # wall_x_range2[1] = wall_x_range1[1]
                        continue

        for id in range(len(wall_vps)):
            wall_vp = wall_vps[id]
            wall_x_range_id, wall_y_range_id = wall_x_range[id], wall_y_range[id]
            wall_x_range_id_new, wall_y_range_id_new = wall_x_range_new[id], wall_y_range_new[id]
            for point in wall_vp:
                if point[0] in wall_x_range_id:
                    index_x = wall_x_range_id.index(point[0])
                    point[0] = wall_x_range_id_new[index_x]
                if point[1] in wall_y_range_id:
                    index_y = wall_y_range_id.index(point[1])
                    point[1] = wall_y_range_id_new[index_y]
        return wall_vps

    wall_vps = wall_thickness_refinement_gongmian(wall_vps)  # 先共面修正 后垂直修正
    wall_vps = wall_thickness_refinement_chuizhi(wall_vps)  # 后垂直修正
    return wall_vps
def get_bounding_box(vertices, height):
    """
    从顶点和高度中提取包围框的最小和最大 x, y, z 坐标
    """
    vertices = np.array(vertices)
    x_min, y_min, z_min = np.min(vertices, axis=0)
    x_max, y_max, z_max = np.max(vertices, axis=0)
    z_max = z_min + height  # 使用高度信息更新 z_max
    return (x_min, y_min, z_min, x_max, y_max, z_max)
def is_bbox_overlap(bbox1, bbox2):
    """
    检查两个包围框是否重叠
    """
    x_min1, y_min1, z_min1, x_max1, y_max1, z_max1 = bbox1
    x_min2, y_min2, z_min2, x_max2, y_max2, z_max2 = bbox2

    # 检查是否有重叠
    overlap_x = (x_min1 < x_max2) and (x_max1 > x_min2)
    overlap_y = (y_min1 < y_max2) and (y_max1 > y_min2)
    overlap_z = (z_min1 < z_max2) and (z_max1 > z_min2)

    return overlap_x and overlap_y and overlap_z
def calculate_overlap_volume(bbox1, bbox2):
    """
    计算两个重叠包围框的重叠体积
    """
    if not is_bbox_overlap(bbox1, bbox2):
        return 0.0

    x_min1, y_min1, z_min1, x_max1, y_max1, z_max1 = bbox1
    x_min2, y_min2, z_min2, x_max2, y_max2, z_max2 = bbox2

    overlap_x_min = max(x_min1, x_min2)
    overlap_y_min = max(y_min1, y_min2)
    overlap_z_min = max(z_min1, z_min2)
    overlap_x_max = min(x_max1, x_max2)
    overlap_y_max = min(y_max1, y_max2)
    overlap_z_max = min(z_max1, z_max2)

    overlap_volume = (overlap_x_max - overlap_x_min) * (overlap_y_max - overlap_y_min) * (
            overlap_z_max - overlap_z_min)
    return overlap_volume
def find_wall_for_element(door_vertices, door_height, wall_data):
    """
    找到门所属的墙体
    """
    door_bbox = get_bounding_box(door_vertices, door_height)
    max_overlap_volume = 0.0
    best_wall = None

    for wall in wall_data:
        wall_bbox = get_bounding_box(wall['element_vps'], wall['height'])
        overlap_volume = calculate_overlap_volume(door_bbox, wall_bbox)
        if overlap_volume > max_overlap_volume:
            max_overlap_volume = overlap_volume
            best_wall = wall

    return best_wall
def adjust_element_to_wall(door_vertices, wall, flag=True):
    """
    调整门的位置和厚度，使其完全嵌入墙体中
    """
    wall_bbox = get_bounding_box(wall['element_vps'], wall['height'])
    x_min_wall, y_min_wall, z_min_wall, x_max_wall, y_max_wall, z_max_wall = wall_bbox

    # 门的新厚度等于墙的厚度
    new_thickness = wall['thickness']

    # 调整门的位置，使其完全嵌入墙体中
    door_vertices = np.array(door_vertices)
    x_min_door, y_min_door, z_min_door = np.min(door_vertices, axis=0)
    x_max_door, y_max_door, z_max_door = np.max(door_vertices, axis=0)

    # 将门的位置调整到墙体内部
    if (x_max_door - x_min_door) > (y_max_door - y_min_door):
        new_x_min_door = x_min_door
        new_y_min_door = y_min_wall
        new_x_max_door = x_max_door
        new_y_max_door = y_max_wall
    else:
        new_x_min_door = x_min_wall
        new_y_min_door = y_min_door
        new_x_max_door = x_max_wall
        new_y_max_door = y_max_door

    if flag:
        new_z_min_door = z_min_wall  # 门底部与墙底部对齐
    else:
        new_z_min_door = z_min_door
    new_door_vertices = [
        [new_x_min_door, new_y_min_door, new_z_min_door],
        [new_x_max_door, new_y_min_door, new_z_min_door],
        [new_x_min_door, new_y_max_door, new_z_min_door],
        [new_x_max_door, new_y_max_door, new_z_min_door],
    ]
    return new_door_vertices, new_thickness
def find_wall_for_wd(element_information):
    # step1：输入door window wall信息
    door_info, window_info, wall_info = element_information['door'], element_information['window'], element_information[
        'wall']
    door_wall_names, window_wall_names = {}, {}
    # step2.1：对 door 找寻 关属墙
    # step2.2：修改door的尺寸 方便嵌入
    for door in door_info:
        door_vertices, door_height = door['element_vps'], door['height']
        best_wall = find_wall_for_element(door_vertices, door_height, wall_info)
        door_wall_names[door['name']] = best_wall['name']
        new_door_vertices, new_thickness = adjust_element_to_wall(door_vertices, best_wall)
        door['element_vps'], door['thickness'] = new_door_vertices, new_thickness
    element_information['door'] = door_info

    # step3.1：对 window 找寻 关属墙
    # step3.2：修改window的尺寸 方便嵌入
    for window in window_info:
        window_vertices, window_height = window['element_vps'], window['height']
        best_wall = find_wall_for_element(window_vertices, window_height, wall_info)
        window_wall_names[window['name']] = best_wall['name']
        new_door_vertices, new_thickness = adjust_element_to_wall(window_vertices, best_wall, flag=False)
        window['element_vps'], window['thickness'] = new_door_vertices, new_thickness
    element_information['window'] = window_info
    return element_information, door_wall_names, window_wall_names
def txt_information(input_file_path):
    with open(input_file_path, "r", encoding="utf-8") as file:
        content = file.readlines()
        wall_name, wall_height_str, wall_thickness_str, wall_length_str, wall_vp_str = content[1].replace('\n', ''), \
        content[3], content[5], content[7], content[9:]
        wall_height = np.array(wall_height_str.replace('\n', '').replace('[', '').replace(']', ''), dtype=float)
        wall_thickness = np.array(wall_thickness_str.replace('\n', '').replace('[', '').replace(']', ''), dtype=float)
        wall_length = np.array(wall_length_str.replace('\n', '').replace('[', '').replace(']', ''), dtype=float)

        wall_vp = []
        for i in wall_vp_str:
            float_list = []
            # 移除方括号并按空格分割字符串
            cleaned_s = i.replace('[', '').replace(']', '').strip()
            if i:
                parts = cleaned_s.split()
                # 将每个部分转换为浮点数并添加到浮点数列表中
                for part in parts:
                    try:
                        float_list.append(float(part))
                    except ValueError as e:
                        print(f"Error converting '{part}' to float: {e}")
                wall_vp.append(float_list)
        file.close()
        wall_vp_temp = np.array(wall_vp)
        wall_vp = []
        for i in wall_vp_temp:
            if i[2] == np.min(wall_vp_temp, axis=0)[2]:
                wall_vp.append(list(i))
    return float(wall_height), float(wall_thickness), float(wall_length), wall_vp, wall_name

def main():
    # 元素尺寸信息读取
    input_data_files = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\整体房间构建\wall_window_door_floor_v2\element_information'
    element_classes = os.listdir(input_data_files)
    element_information = {element: [] for element in element_classes}

    # /**********************************************先读取墙体的信息并进行优化**********************************************/
    def find_first_wall_element(data):
        for element in data:
            if isinstance(element, str) and element.startswith('wall'):
                return element
        return None  # 如果没有找到，返回 None
    element_walls = find_first_wall_element(element_classes)
    if element_walls != None:
        element_walls_data_paths = os.path.join(input_data_files, element_walls)
        element_walls_vps = []
        for element_walls_data_path in os.listdir(element_walls_data_paths):
            element_wall_data_path = os.path.join(element_walls_data_paths, element_walls_data_path)
            height, thickness, length, element_vp, element_name = txt_information(element_wall_data_path)
            temp = {}
            temp['name'], temp['height'], temp['thickness'], temp['length'] = element_name, height, thickness, length
            element_information[element_walls].append(temp)
            element_walls_vps.append(element_vp)
        # element_walls_vps = wall_refinement(element_walls_vps)
        for i in range(len(element_walls_vps)):
            element_information[element_walls][i]['element_vps'] = element_walls_vps[i]
    else: sys.exit("存在错误")

    # /*********************************window door floor找到对应墙体并进行尺寸优化******************************************/
    # 首先提取 door window floor的信息
    for classes in element_classes:
        if classes.startswith('wall'): continue
        input_class_data_paths = os.path.join(input_data_files, classes)
        for input_class_data_path in os.listdir(input_class_data_paths):
            input_class_data_path = os.path.join(input_class_data_paths, input_class_data_path)
            height, thickness, length, element_vp, element_name = txt_information(input_class_data_path)
            temp = {}
            temp['name'], temp['height'], temp['thickness'], temp['length'] = element_name, height, thickness, length
            temp['element_vps'] = element_vp
            element_information[classes].append(temp)
    # 针对 door window找寻 最近的wall（记住这一步不适用于floor）
    element_information_ref, door_wall_names, window_wall_names = find_wall_for_wd(element_information)
    # 针对 floor针对墙体尺寸优化代码（之后）
    # 生成ifc模型
    output_file = '1.ifc'
    ifc_generation_based_on_info(element_information_ref, door_wall_names, window_wall_names, output_file)



if __name__ == '__main__':
    main()
