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
from IfcOpenHouse.opening_data import door_params, single_window_params, triple_window_params
from IfcOpenHouse.ios_utils import (
    IfcOpenShellPythonAPI, placement_matrix, clipping, ColourRGB, TerrainBuildMethod,
    build_native_bspline_terrain, build_tesselated_occ_terrain, ios_entity_overwrite_hook
)
import uuid

def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)

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

    # wall_Parameters
    length = 5.0
    height = 3.0
    thickness = 0.2
    wall = ios.root.create_entity(file, ifc_class='IfcWall', name='South wall', predefined_type='SOLIDWALL')
    ios.spatial.assign_container(file, product=wall, relating_structure=storey);
    south_wall_representation = ios.geometry.add_wall_representation(
        file, context=body, length=length, height=height,
        thickness=thickness
    )
    ios.geometry.assign_representation(file, product=wall, representation=south_wall_representation)
    corner_point = [0.0, 0.0, 0.0]
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]
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
    wall.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )
    wall.ObjectPlacement = local_placement
    #
    # # # door_Parameters 先开洞
    length = 1.0
    height = 2.0
    thickness = 0.2
    door_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
    door_opening_representation = ios.geometry.add_wall_representation(
        file, context=body, length=length, height=height,
        thickness=thickness
    )
    ios.geometry.assign_representation(
        file, product=door_opening, representation=door_opening_representation
    )
    corner_point = [1.0, 0.0, 0.0]
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]
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
    door_opening.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=wall.ObjectPlacement,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )

    door_opening.ObjectPlacement = local_placement
    ios.void.add_opening(file, opening=door_opening, element=wall)
    # # 在创建门
    door = ios.root.create_entity(file, ifc_class='IfcDoor', name='Main door', predefined_type='DOOR')
    ios.spatial.assign_container(file, product=door, relating_structure=storey)
    length = 1.0
    height = 2.0
    thickness = 0.2
    door_representation = ios.geometry.add_door_representation(  # requires mathutils
        file, context=body, length=length, height=height,thickness=thickness)
    corner_point = [1.0, 0.0, 0.0]
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]
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
    door.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )
    ios.geometry.assign_representation(file, product=door, representation=door_representation)
    door.ObjectPlacement = local_placement

    ios.void.add_filling(file, opening=door_opening, element=door)

    # window_Parameters 先开洞
    length = 1.0
    height = 1.0
    thickness = 0.2
    window_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
    window_opening_representation = ios.geometry.add_wall_representation(
        file, context=body, length=length, height=height,
        thickness=thickness
    )
    ios.geometry.assign_representation(
        file, product=window_opening, representation=window_opening_representation
    )
    corner_point = [3.0, 0.0, 1.0]
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]
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
    window_opening.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((0.0, 0.0, 0.0)),
            Axis=None,
            RefDirection=None
        )
    )

    window_opening.ObjectPlacement = local_placement
    ios.void.add_opening(file, opening=window_opening, element=wall)
    # # 在创建window
    common_window_params = {
        'overall_height': 1,
        'lining_properties': {
            'LiningDepth': 0.05,
            'LiningThickness': 0.05,
            'LiningOffset': 0.05,
            'LiningToPanelOffsetX': 0.025,
            'LiningToPanelOffsetY': 0.025,
            'MullionThickness': 0.05,
            'FirstMullionOffset': 5.5 / 3,
            'SecondMullionOffset': 2 * 5.5 / 3,
            'TransomThickness': 0.05,
            'FirstTransomOffset': 0.,
            'SecondTransomOffset': 0.,
        }
    }

    window_panel_properties = {
        'FrameDepth': 0.035,
        'FrameThickness': 0.2
    }

    single_window_params = {
        'partition_type': 'SINGLE_PANEL',
        'overall_width': 1,
        'panel_properties': [window_panel_properties],
        **common_window_params
    }
    window = ios.root.create_entity(file, ifc_class='IfcWindow', name='Main door', predefined_type='WINDOW')
    window.PartitioningType = single_window_params['partition_type']
    ios.spatial.assign_container(file, product=window, relating_structure=storey)
    length = 1.0
    height = 1.0
    thickness = 0.2
    window_representation = ios.geometry.add_window_representation(
        file, context=body, **single_window_params
    )
    # ios.geometry.assign_representation(file, product=window, representation=window_representation)
    corner_point = [3.0, 0.0, 1.0]
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]

    # polyline = file.createIfcPolyline([file.createIfcCartesianPoint(p) for p in points])
    # profile = file.createIfcArbitraryClosedProfileDef('AREA', None, polyline)
    # extrusion_direction = file.createIfcDirection([0.0, 0.0, 1.0])
    # extruded_solid = file.createIfcExtrudedAreaSolid(profile, None, extrusion_direction, height)
    # representation = file.createIfcShapeRepresentation(
    #     ContextOfItems=body,
    #     RepresentationIdentifier='Body',
    #     RepresentationType='SweptSolid',
    #     Items=[extruded_solid]
    # )
    # product_representation = file.createIfcProductDefinitionShape(Representations=[representation])
    # window.Representation = product_representation
    local_placement = file.createIfcLocalPlacement(
        PlacementRelTo=None,
        RelativePlacement=file.createIfcAxis2Placement3D(
            Location=file.createIfcCartesianPoint((3.0, 0.0, 1.0)),
            Axis=None,
            RefDirection=None
        )
    )
    window.ObjectPlacement = local_placement
    ios.geometry.assign_representation(
        file, product=window, representation=window_representation
    )
    ios.void.add_filling(file, opening=window_opening, element=window)
    file.write(output_file)




if __name__ == "__main__":
    main()
