import os

import ifcopenshell
import uuid

import numpy as np

def wall_thickness_refinement_gongmian(wall_vps, threshold = 0.475):
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
        for id2 in range(id1+1, len(wall_y_range)):
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

def wall_thickness_refinement_chuizhi(wall_vps, threshold = 0.5): #(有三种不同状态改)
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
            #镶嵌状态
            if (wall_y_range1[1] - wall_y_range1[0] > threshold) & (wall_x_range2[1] - wall_x_range2[0] > threshold):
            #镶嵌状态
                if (wall_x_range1[1] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):# 镶嵌1
                    wall_y_range1[1] = wall_y_range2[1]
                    continue
                elif (wall_x_range2[1] > wall_x_range1[1]) & (wall_x_range1[0] > wall_x_range2[0]) & (wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):# 镶嵌2
                    wall_y_range1[1] = wall_y_range2[1]
                    continue
                elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] >= wall_x_range1[0]) & (wall_y_range2[1] > wall_y_range1[1]) & (wall_y_range1[1] > wall_y_range2[0]):# 镶嵌3
                    # wall_x_range2[0] = wall_x_range1[0]
                    wall_y_range1[1] = wall_y_range2[1]
                    continue
                    # 镶嵌状态
                elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] > wall_x_range1[0]) & (wall_y_range1[1] > wall_y_range2[1]) & (wall_y_range2[0] > wall_y_range1[0]):  # 镶嵌4
                    # wall_x_range2[0] = wall_x_range1[0]
                    continue
                elif (wall_x_range1[1] > wall_x_range2[0]) & (wall_x_range2[0] > wall_x_range1[0]) & (wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌5
                    wall_y_range1[0] = wall_y_range2[0]
                    continue
                elif (wall_x_range2[1] > wall_x_range1[1]) & (wall_x_range1[0] > wall_x_range2[0]) & (wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌6
                    wall_y_range1[0] = wall_y_range2[0]
                    continue
                elif (wall_x_range1[1] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (wall_y_range2[1] > wall_y_range1[0]) & (wall_y_range1[0] > wall_y_range2[0]):  # 镶嵌7
                    wall_y_range1[0] = wall_y_range2[0]
                    # wall_x_range2[1] = wall_x_range1[1]
                    continue
                elif (wall_x_range1[0] > wall_x_range2[1]) & (wall_x_range2[1] > wall_x_range1[0]) & (wall_y_range1[1] > wall_y_range2[1]) & (wall_y_range2[0] > wall_y_range1[0]):  # 镶嵌8
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
def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)

def create_wall(ifc_file, context, corner_point, length, height, thickness, name):
    p1 = [float(corner_point[0][0]), float(corner_point[0][1]), float(corner_point[0][2])]
    p2 = [float(corner_point[1][0]), float(corner_point[1][1]), float(corner_point[1][2])]
    p3 = [float(corner_point[2][0]), float(corner_point[2][1]), float(corner_point[2][2])]
    p4 = [float(corner_point[3][0]), float(corner_point[3][1]), float(corner_point[3][2])]

    points = [p2, p1, p3, p4]

    polyline = ifc_file.createIfcPolyline([ifc_file.createIfcCartesianPoint(p) for p in points])
    profile = ifc_file.createIfcArbitraryClosedProfileDef('AREA', None, polyline)
    extrusion_direction = ifc_file.createIfcDirection([0.0, 0.0, 1.0])
    extruded_solid = ifc_file.createIfcExtrudedAreaSolid(profile, None, extrusion_direction, height)
    wall = ifc_file.createIfcWallStandardCase(GlobalId=create_guid(), Name=name)
    representation = ifc_file.createIfcShapeRepresentation(
        ContextOfItems=context,
        RepresentationIdentifier='Body',
        RepresentationType='SweptSolid',
        Items=[extruded_solid]
    )
    product_representation = ifc_file.createIfcProductDefinitionShape(Representations=[representation])
    wall.Representation = product_representation
    local_placement = ifc_file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=ifc_file.createIfcAxis2Placement3D(
            Location=ifc_file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )
    wall.ObjectPlacement = local_placement
    return wall

def txt2ifc(wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names,output_path):
    ifc_file = ifcopenshell.file()

    # Create project and site
    project = ifc_file.createIfcProject(GlobalId=create_guid(), Name="Overlapping Walls Project")
    context = ifc_file.createIfcGeometricRepresentationContext(
        ContextIdentifier='Model',
        ContextType='Model',
        CoordinateSpaceDimension=3,
        Precision=0.00001, # 0.002
        WorldCoordinateSystem=ifc_file.createIfcAxis2Placement3D(ifc_file.createIfcCartesianPoint((0.0, 0.0, 0.0))),
        TrueNorth=None
    )
    site = ifc_file.createIfcSite(GlobalId=create_guid(), Name="Default Site")
    ifc_file.createIfcRelAggregates(GlobalId=create_guid(), RelatingObject=project, RelatedObjects=[site])
    total_wall = []

    for wall_id in range(len(wall_lengths)):
        # Para.
        length = wall_lengths[wall_id]
        height = wall_heights[wall_id]
        wall_vp = wall_vps[wall_id]

        thickness = wall_thicknesses[wall_id]
        wall = create_wall(ifc_file, context, wall_vp, length, height, thickness, wall_names[wall_id])
        total_wall.append(wall)
    ifc_file.createIfcRelContainedInSpatialStructure(
        GlobalId=create_guid(),
        RelatingStructure=site,
        RelatedElements=total_wall
    )
    ifc_file.write(output_path)
    print(output_path)
def txt_information(input_file_path):
    with open(input_file_path, "r", encoding= "utf-8") as file:
        content = file.readlines()
        wall_name, wall_height_str, wall_thickness_str, wall_length_str, wall_vp_str = content[1].replace('\n', ''), content[3], content[5], content[7], content[9:]
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
    return float(wall_height), float(wall_thickness), float(wall_length),wall_vp, wall_name+'.ifc'


# 示例使用
if __name__ == '__main__':
    input_file_paths = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\wall\wall_rh\data_wall_final_information_notrefinement'  # 输入点云文件路径
    out_file_dir = r'E:\scientific_research\code\pycharm_code\PointCloud\点云逆向建模\PC_I\PCD2BIM_paper\wall\wall_ifc\res'
    wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names = [], [], [], [], []
    for input_file_path in os.listdir(input_file_paths):
        input_file_path = os.path.join(input_file_paths, input_file_path)
        wall_height, wall_thickness, wall_length, wall_vp, wall_name = txt_information(input_file_path)
        wall_heights.append(wall_height), wall_thicknesses.append(wall_thickness)
        wall_lengths.append(wall_length), wall_vps.append(wall_vp), wall_names.append(wall_name)
    out_file_path = os.path.join(out_file_dir, wall_name.split('-')[0]+wall_name.split('-')[1]+'.ifc')
    # wall_vps = wall_thickness_refinement_gongmian(wall_vps)
    # wall_vps = wall_thickness_refinement_chuizhi(wall_vps)
    txt2ifc(wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names, out_file_path)

