import os

import ifcopenshell
import uuid
import math
import numpy as np


def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)


def find_nearest_point(points):
    # 初始条件，假设第一个点为我们要找的点，并计算其与原点的距离
    min_point = points[0]
    min_distance = math.sqrt(min_point[0] ** 2 + min_point[1] ** 2 + min_point[2] ** 2)

    for point in points[1:]:
        distance = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)
        if distance < min_distance:
            min_point = point
            min_distance = distance

    return min_point

def create_wall(ifc_file, context, corner_point, length, height, thickness, name):
    p1 = [float(corner_point[0][0]), float(corner_point[0][1]), float(corner_point[0][2])]
    p2 = [float(corner_point[1][0]), float(corner_point[1][1]), float(corner_point[1][2])]
    p3 = [float(corner_point[2][0]), float(corner_point[2][1]), float(corner_point[2][2])]
    p4 = [float(corner_point[3][0]), float(corner_point[3][1]), float(corner_point[3][2])]
    p5 = [float(corner_point[4][0]), float(corner_point[4][1]), float(corner_point[4][2])]
    p6 = [float(corner_point[5][0]), float(corner_point[5][1]), float(corner_point[5][2])]
    p7 = [float(corner_point[6][0]), float(corner_point[6][1]), float(corner_point[6][2])]
    p8 = [float(corner_point[7][0]), float(corner_point[7][1]), float(corner_point[7][2])]

    points = [p1, p2, p3, p4, p5, p6, p7, p8]

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
        # 在 x 最小的点中，找到 y 也最小的点
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
        wall_vp = np.array(wall_vp)
    return float(wall_height), float(wall_thickness), float(wall_length),wall_vp, wall_name+'.ifc'


# 示例使用
if __name__ == '__main__':
    input_file_paths = r'D:\ctz\code\PC_I\PCD2BIM_paper\wall_rh\data_wall_final_information'  # 输入点云文件路径
    out_file_dir = r'D:\ctz\code\PC_I\PCD2BIM_paper\wall_ifc\res'
    wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names = [], [], [], [], []
    for input_file_path in os.listdir(input_file_paths):
        input_file_path = os.path.join(input_file_paths, input_file_path)
        wall_height, wall_thickness, wall_length, wall_vp, wall_name = txt_information(input_file_path)
        wall_heights.append(wall_height), wall_thicknesses.append(wall_thickness)
        wall_lengths.append(wall_length), wall_vps.append(wall_vp), wall_names.append(wall_name)
    out_file_path = os.path.join(out_file_dir, wall_name.split('-')[0]+wall_name.split('-')[1]+'.ifc')
    txt2ifc(wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names, out_file_path)

