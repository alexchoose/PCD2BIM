import sys
from pathlib import Path
from collections import defaultdict
from mathutils import Vector
import numpy as np
import ifcopenshell
import ifcopenshell.api
import ifcopenshell.api.owner
import ifcopenshell.api.owner.settings
import ifcopenshell.api.material
import ifcopenshell.api.geometry
import ifcopenshell.validate

from IfcOpenHouse.ios_utils import (
    IfcOpenShellPythonAPI, placement_matrix, clipping, ColourRGB, TerrainBuildMethod,
    build_native_bspline_terrain, build_tesselated_occ_terrain, ios_entity_overwrite_hook
)
import uuid


def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)
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
def ifc_representens(file, body,element, corner_point, height):
    points = sort_points(corner_point)
    polyline = file.createIfcPolyline([file.createIfcCartesianPoint(p) for p in points])
    profile = file.createIfcArbitraryClosedProfileDef('AREA', None, polyline)
    extrusion_direction = file.createIfcDirection([0.0, 0.0, 1.0])
    extruded_solid = file.createIfcExtrudedAreaSolid(profile, None, extrusion_direction, height)
    representation = file.createIfcShapeRepresentation(
        ContextOfItems=body,
        RepresentationIdentifier='Body',
        RepresentationType='SweptSolid',
        Items=[extruded_solid]
    )
    product_representation = file.createIfcProductDefinitionShape(Representations=[representation])
    element.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )
    element.ObjectPlacement = local_placement
    return element

def create_ifc_wall(ios, file, storey, body, corner_point, height, name):
    # wall_Parameters
    wall = ios.root.create_entity(file, ifc_class='IfcWall', name=name, predefined_type='SOLIDWALL')
    ios.spatial.assign_container(file, product=wall, relating_structure=storey)
    # position
    wall = ifc_representens(file, body, wall, corner_point, height)
    return wall

def create_ifc_footing(ios, file, storey, body, corner_point, height, name):
    # footing_Parameters
    footing = ios.root.create_entity(file, ifc_class='IfcFooting', name=name, predefined_type='STRIP_FOOTING')
    ios.spatial.assign_container(file, product=footing, relating_structure=storey)
    # position
    footing = ifc_representens(file, body, footing, corner_point, height)

    return footing

def create_ifc_door(ios, file, storey, body, corner_point, height, name, wall):
    # # # door_Parameters 先开洞
    door_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
    #对墙体进行 开洞 （位置）
    door_opening = ifc_representens(file, body, door_opening, corner_point, height)
    ios.void.add_opening(file, opening=door_opening, element=wall)
    #建立door模型
    door = ios.root.create_entity(file, ifc_class='IfcDoor', name=name, predefined_type='DOOR')
    ios.spatial.assign_container(file, product=door, relating_structure=storey)
    door = ifc_representens(file, body, door, corner_point, height)
    #填充opening
    ios.void.add_filling(file, opening=door_opening, element=door)
    return door

def create_ifc_window(ios, file, storey, body, corner_point, height, name, wall):
    # # # window_Parameters 先开洞
    window_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
    #对墙体进行 开洞 （位置）
    window_opening = ifc_representens(file, body, window_opening, corner_point, height)
    ios.void.add_opening(file, opening=window_opening, element=wall)
    #建立window模型
    window = ios.root.create_entity(file, ifc_class='IfcWindow', name=name, predefined_type='WINDOW')
    ios.spatial.assign_container(file, product=window, relating_structure=storey)
    window = ifc_representens(file, body, window, corner_point, height)
    #填充opening
    ios.void.add_filling(file, opening=window_opening, element=window)
    return window

def main(output_file=r'overlapping_walls_with_opening.ifc'):
    # Data definition
    project_name = 'IFC Open House'
    author_details = {'given_name': 'Carlos', 'family_name': 'V', 'identification': 'CV'}
    organization_details = {'name': 'OSArch', 'identification': 'OSArch'}
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


    file.write(output_file)

if __name__ == "__main__":
    main()
