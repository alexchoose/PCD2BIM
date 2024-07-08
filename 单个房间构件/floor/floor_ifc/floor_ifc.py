import os
import ifcopenshell
import uuid
import numpy as np
import math
def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)
# 按照连线的方式进行排序
def sort_points(points):
    sorted_points = []
    current_point = points[0]
    points_copy = points.copy()

    while points_copy:
        sorted_points.append(current_point)
        points_copy.remove(current_point)

        # 按顺序寻找下一个点
        if points_copy:
            # 向上
            up_points = [p for p in points_copy if p[0] == current_point[0] and p[1] > current_point[1]]
            if up_points:
                current_point = min(up_points, key=lambda p: p[1])
                continue

            # 向右
            right_points = [p for p in points_copy if p[1] == current_point[1] and p[0] > current_point[0]]
            if right_points:
                current_point = min(right_points, key=lambda p: p[0])
                continue

            # 向下
            down_points = [p for p in points_copy if p[0] == current_point[0] and p[1] < current_point[1]]
            if down_points:
                current_point = max(down_points, key=lambda p: p[1])
                continue

            # 向左
            left_points = [p for p in points_copy if p[1] == current_point[1] and p[0] < current_point[0]]
            if left_points:
                current_point = max(left_points, key=lambda p: p[0])
                continue
    final_ifc_points = []
    for sorted_point in sorted_points:
        temp = [float(sorted_point[0]), float(sorted_point[1]), float(sorted_point[2])]
        final_ifc_points.append(temp)
    return final_ifc_points
def create_floor(ifc_file, context, corner_points, length, height, thickness, name):

    points = sort_points(corner_points)

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
        wall = create_floor(ifc_file, context, wall_vp, length, height, thickness, wall_names[wall_id])
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
    input_file_paths = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\单个房间构件\floor\floor_ifc\information'  # 输入点云文件路径
    out_file_dir = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\单个房间构件\floor\floor_ifc'
    wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names = [], [], [], [], []
    for input_file_path in os.listdir(input_file_paths):
        input_file_path = os.path.join(input_file_paths, input_file_path)
        wall_height, wall_thickness, wall_length, wall_vp, wall_name = txt_information(input_file_path)
        wall_heights.append(wall_height), wall_thicknesses.append(wall_thickness)
        wall_lengths.append(wall_length), wall_vps.append(wall_vp), wall_names.append(wall_name)
    out_file_path = os.path.join(out_file_dir, 'floor.ifc')
    # wall_vps = wall_thickness_refinement_gongmian(wall_vps)
    # wall_vps = wall_thickness_refinement_chuizhi(wall_vps)
    txt2ifc(wall_heights, wall_thicknesses, wall_lengths, wall_vps, wall_names, out_file_path)

