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

##/********************************项目数据定义********************************/
# Data definition
project_name = 'IFC Open House'
author_details = {'given_name': 'Carlos', 'family_name': 'V', 'identification': 'CV'}
organization_details = {'name': 'OSArch', 'identification': 'OSArch'}
site_name, building_name, storey_name = 'OSArch Land', 'Open house', 'Ground floor'

# All dimensions in meters
storey_size = Vector([10., 5., 3.])
wall_thickness = 0.36
footing_ledge = 0.05
footing_size = Vector([
    storey_size.x + 2 * (wall_thickness + footing_ledge),
    storey_size.y + 2 * (wall_thickness + footing_ledge),
    2.
])
roof_ledge = Vector([0.1, 0.22])
roof_thickness = 0.36
roof_angle = 45. # degrees
roof_angle_sin = float(np.sin(roof_angle * np.pi/180))
roof_angle_cos = float(np.cos(roof_angle * np.pi/180))
roof_height = float(
    (storey_size.y / 2 + wall_thickness + roof_ledge.y) * np.tan(roof_angle * np.pi / 180)
)
roof_size = Vector([
    storey_size.x + 2 * (wall_thickness + roof_ledge.x),
    storey_size.y + 2 * (wall_thickness + roof_ledge.y),
    roof_height
])
door_horizontal_offset = 1.6
window_base_height = 0.4
right_window_horizontal_offset = 2.
stair_width = 1.2

# Colours for surface styles
wall_colour = ColourRGB(.75, 0.73, 0.68)
footing_colour = ColourRGB(.38, 0.4, 0.42)
roof_colour = ColourRGB(.24, 0.08, 0.04)
terrain_colour = ColourRGB(.15, 0.25, 0.05)
door_colour = ColourRGB(.8, .8, .8)
window_colour = ColourRGB(.5, 0.4, 0.3, transparency=0.8)
stair_colour = ColourRGB(.45, 0.47, 0.56)

# Choice of terrain building method. Use NONE if unsure about the viewer capabilities.
terrain_build_method = TerrainBuildMethod.TESSELATE_OCC_SHAPE

# Door and window geometric info is defined in a separate file due to its complexity
from IfcOpenHouse.opening_data import door_params, single_window_params, triple_window_params

##/********************************绑定api********************************/
# Little trickery to ease the use of the ifcopenshell.api when scripting
ios = IfcOpenShellPythonAPI()  #q1: thoughts about a data-scientish "import ifcopenshell.api as ios"?

# standard use      -> ifcopenshell.api.run('root.create_entity', file, ifc_class='IfcWall')
# with the trickery -> ios.root.create_entity(file, ifc_class='IfcWall')

# It will reduce the overall string overhead, as well as the length of the API calls
# However, it will not help with static typing autocomplete and help
# Bear in mind that currently, this is not a canonical use of IfcOpenShell


# Setting up the project
file = ios.project.create_file(version='IFC4')
# Don't use 2X3 in 2023! It's terribly outdated and lacks many useful classes. This simple
# project uses many >=IFC4 features, and hence selecting 'IFC2X3' here would only lead to issues.
# Pending to use 4x3 (much better docs) when ios defaults to IFC4X3_TC1 and IFC.js supports it

project = ios.root.create_entity(file, ifc_class='IfcProject', name=project_name)
ios.project.assign_declaration(file, definition=project, relating_context=project)  #q2: from my ignorance, is this necessary?
ios.unit.assign_unit(
    file, length={'is_metric': True, 'raw': 'METERS'}, area={'is_metric': True, 'raw': 'METERS'},
    volume={'is_metric': True, 'raw': 'METERS'}
)
ctx = ios.context.add_context(file, context_type='Model')
body = ios.context.add_context(  #q3: isn't this screaming for "context.add_subcontext"? also, context_type may be redundant
    file, context_type='Model', context_identifier='Body', target_view='MODEL_VIEW', parent=ctx
)

# We allow for live overwriting of IfcOpenShell entities within the notebook environment
# Only use this sorcery when experimenting in Jupyter Notebooks, never in production
sys.addaudithook(
    ios_entity_overwrite_hook(file, sys.modules[__name__], do_not_delete=[project, ctx, body])
)

#/********************************定义整体房间与楼层********************************/

site = ios.root.create_entity(file, ifc_class='IfcSite', name=site_name)
ios.aggregate.assign_object(file, product=site, relating_object=project)
building = ios.root.create_entity(file, ifc_class='IfcBuilding', name=building_name)
ios.aggregate.assign_object(file, product=building, relating_object=site)
storey = ios.root.create_entity(file, ifc_class='IfcBuildingStorey', name=storey_name)
ios.aggregate.assign_object(file, product=storey, relating_object=building);
#定义整体范围
pset_site_common = ios.pset.add_pset(file, product=site, name='Pset_SiteCommon')
ios.pset.edit_pset(file, pset=pset_site_common, properties={'TotalArea': 400.})
#/****************************定义floor**********************************/
#footing
footing = ios.root.create_entity(file, ifc_class='IfcFooting', name='Footing', predefined_type='STRIP_FOOTING')
ios.spatial.assign_container(file, product=footing, relating_structure=storey)
footing_representation = ios.geometry.add_wall_representation(
    file, context=body, length=footing_size.x, height=footing_size.z, thickness=footing_size.y
)
ios.geometry.assign_representation(file, product=footing, representation=footing_representation)
ios.geometry.edit_object_placement(
    file, product=footing, matrix=placement_matrix(
        [-footing_size.x/2, -wall_thickness/2 - footing_ledge, -footing_size.z]
    )
)
footing_style = ios.style.add_style(file)
ios.style.add_surface_style(
    file, style=footing_style, ifc_class='IfcSurfaceStyleShading', attributes=footing_colour.info
)
ios.style.assign_representation_styles(
    file, shape_representation=footing_representation, styles=[footing_style]
);
#/********************************定义墙开口等********************************/
#south wall
south_wall = ios.root.create_entity(file, ifc_class='IfcWall', name='South wall', predefined_type='SOLIDWALL')
ios.spatial.assign_container(file, product=south_wall, relating_structure=storey);
south_wall_representation = ios.geometry.add_wall_representation(
    file, context=body, length=storey_size.x + 2 * wall_thickness, height=storey_size.z,
    thickness=wall_thickness
)
ios.geometry.assign_representation(file, product=south_wall, representation=south_wall_representation)
ios.geometry.edit_object_placement(
    file, product=south_wall, matrix=placement_matrix(
        [-storey_size.x / 2 - wall_thickness, -wall_thickness / 2, 0.]
    )
);  #q4: why a matrix if Y is going to be ignored? why not just pass the placement coords + optionals x_local, z_local and scale?
wall_style = ios.style.add_style(file)
ios.style.add_surface_style(
    file, style=wall_style, ifc_class='IfcSurfaceStyleShading', attributes=wall_colour.info
)
ios.style.assign_representation_styles(
    file, shape_representation=south_wall_representation, styles=[wall_style]
);
#west wall with windows
west_void_margin = 0.5
west_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
west_opening_width = 2 * single_window_params['overall_width']
wo_representation = ios.geometry.add_wall_representation(
    file, context=body,
    length=triple_window_params['overall_width'] + west_void_margin,
    height=triple_window_params['overall_height'],
    thickness=west_opening_width
)
ios.geometry.assign_representation(file, product=west_opening, representation=wo_representation)
west_opening_coords = [
    (
        -storey_size.x / 2 - wall_thickness - west_void_margin
        + single_window_params['lining_properties']['LiningOffset']
    ),
    (
        -west_opening_width / 2 - wall_thickness / 3
        + triple_window_params['lining_properties']['LiningOffset']
        + triple_window_params['lining_properties']['LiningDepth']
    ),
    window_base_height
]
ios.geometry.edit_object_placement(
    file, product=west_opening, matrix=placement_matrix(west_opening_coords)
)
ios.void.add_opening(file, opening=west_opening, element=south_wall)

south_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
south_opening_width = 3.
so_representation = ios.geometry.add_wall_representation(
    file, context=body, length=single_window_params['overall_width'],
    height=single_window_params['overall_height'], thickness=south_opening_width
)
ios.geometry.assign_representation(file, product=south_opening, representation=so_representation)
ios.geometry.edit_object_placement(
    file, product=south_opening, matrix=placement_matrix(
        [right_window_horizontal_offset, -south_opening_width / 2, window_base_height]
    )
)
ios.void.add_opening(file, opening=south_opening, element=south_wall);

# north_wall
north_wall_representation = ifcopenshell.util.element.copy_deep(file, south_wall_representation)
north_wall = ios.root.create_entity(file, ifc_class='IfcWall', name='North wall', predefined_type='SOLIDWALL')
ios.spatial.assign_container(file, product=north_wall, relating_structure=storey)
ios.geometry.assign_representation(file, product=north_wall, representation=north_wall_representation)
ios.geometry.edit_object_placement(
    file, product=north_wall, matrix=placement_matrix(
        [-storey_size.x/2 - wall_thickness, storey_size.y + wall_thickness / 2, 0.]
    )
)
ios.style.assign_representation_styles(
    file, shape_representation=north_wall_representation, styles=[wall_style]
);

east_wall = ios.root.create_entity(
    file, ifc_class='IfcWall', name='East wall', predefined_type='SOLIDWALL'
)
ios.spatial.assign_container(file, product=east_wall, relating_structure=storey)

south_roof_clipping = clipping(
    [0., wall_thickness / 2, storey_size.z], x_dir=[1., 0., 0.],
    z_dir=[0., -roof_angle_sin, roof_angle_cos]
)
north_roof_clipping = clipping(
    [0., storey_size.y + 3 / 2 * wall_thickness, storey_size.z], x_dir=[1., 0., 0.],
    z_dir=[0., roof_angle_sin, roof_angle_cos]
)
# east_wall

east_wall_representation = ios.geometry.add_wall_representation(
    file, context=body, length=wall_thickness, height=storey_size.z + roof_size.z,
    thickness=storey_size.y + 2 * wall_thickness, clippings=[south_roof_clipping, north_roof_clipping]
)

ios.geometry.assign_representation(file, product=east_wall, representation=east_wall_representation)
ios.geometry.edit_object_placement(
    file, product=east_wall, matrix=placement_matrix([storey_size.x / 2, -wall_thickness / 2, 0.])
)

ios.style.assign_representation_styles(
    file, shape_representation=east_wall_representation, styles=[wall_style]
);
west_wall = ios.root.create_entity(
    file, ifc_class='IfcWall', name='West wall', predefined_type='SOLIDWALL'
)
ios.spatial.assign_container(file, product=west_wall, relating_structure=storey)

west_wall_representation = ifcopenshell.util.element.copy_deep(file, east_wall_representation)
ios.geometry.assign_representation(file, product=west_wall, representation=west_wall_representation)
ios.geometry.edit_object_placement(
    file, product=west_wall, matrix=placement_matrix(
        [-storey_size.x / 2 - wall_thickness, -wall_thickness / 2, 0.]
    )
)

west_opening_copy = ifcopenshell.util.element.copy_deep(file, west_opening)
ios.geometry.edit_object_placement(
    file, product=west_opening_copy, matrix=placement_matrix(west_opening_coords)
)
ios.void.add_opening(file, opening=west_opening_copy, element=west_wall)

ios.style.assign_representation_styles(
    file, shape_representation=west_wall_representation, styles=[wall_style]
);

connection_args = {'relating_connection': 'ATEND', 'related_connection': 'ATSTART'}

rel_connect_paths = [
    ios.geometry.connect_path(
        file, relating_element=south_wall, related_element=east_wall, **connection_args
    ),
    ios.geometry.connect_path(
        file, relating_element=east_wall, related_element=north_wall, **connection_args
    ),
    ios.geometry.connect_path(
        file, relating_element=north_wall, related_element=west_wall, **connection_args
    ),
    ios.geometry.connect_path(
        file, relating_element=west_wall, related_element=south_wall, **connection_args
    )
]

#q7: do IfcRelConnectsPathElements with ConnectionGeometry work in any viewer? is this done like this?
# Original IfcOpenHouse had half a wall thickness less of extension per wall end, I bet it's better for
# qto's, but how is the way to properly make connections and to have a proper viz of them in IFC?
point_list = file.create_entity('IfcCartesianPointList2D', CoordList = [[-1., -1.], [1., 1.]])
curve_on_relating = file.create_entity('IfcIndexedPolyCurve', Points=point_list)
connection_curve = file.create_entity(
    'IfcConnectionCurveGeometry', CurveOnRelatingElement=curve_on_relating
)

for path in rel_connect_paths:
    path.ConnectionGeometry = connection_curve

#/********************************定义门等********************************/
door = ios.root.create_entity(file, ifc_class='IfcDoor', name='Main door', predefined_type='DOOR')
ios.spatial.assign_container(file, product=door, relating_structure=storey)

door_opening = ios.root.create_entity(file, ifc_class='IfcOpeningElement')
door_opening_representation = ios.geometry.add_wall_representation(
    file, context=body, length=door_params['overall_width'], height=door_params['overall_height'],
    thickness=door_params['overall_width']
)
ios.geometry.assign_representation(
    file, product=door_opening, representation=door_opening_representation
)
ios.geometry.edit_object_placement(
    file, product=door_opening, matrix=placement_matrix(
        [storey_size.x / 2 - door_params['overall_width'] / 2 , door_horizontal_offset, 0.]
    )
)
ios.void.add_opening(file, opening=door_opening, element=east_wall)

door_representation = ios.geometry.add_door_representation(  # requires mathutils
    file, context=body, **door_params
)
ios.geometry.edit_object_placement(
    file, product=door, matrix=placement_matrix(
        [storey_size.x / 2 + wall_thickness / 4, door_horizontal_offset, 0.],
        x_local=[0., 1., 0.]  # door is rotated into the east wall
    )
)
ios.geometry.assign_representation(file, product=door, representation=door_representation)
ios.void.add_filling(file, opening=door_opening, element=door)

door_style = ios.style.add_style(file)
ios.style.add_surface_style(
    file, style=door_style, ifc_class='IfcSurfaceStyleShading', attributes=door_colour.info
)
ios.style.assign_representation_styles(
    file, shape_representation=door_representation, styles=[door_style]
);

#/********************************定义窗户等********************************/
window_right = ios.root.create_entity(
    file, ifc_class='IfcWindow', name='Right window', predefined_type='WINDOW'
)
window_right.PartitioningType = single_window_params['partition_type']  #q6: couldn't this fit into the previous call?
window_right_representation = ios.geometry.add_window_representation(  # requires mathutils
    file, context=body, **single_window_params
)
ios.spatial.assign_container(file, product=window_right, relating_structure=storey)
ios.geometry.edit_object_placement(
    file, product=window_right, matrix=placement_matrix(
        [right_window_horizontal_offset, -wall_thickness / 3, window_base_height]
    )
)
ios.geometry.assign_representation(
    file, product=window_right, representation=window_right_representation
)
ios.void.add_filling(file, opening=south_opening, element=window_right)

window_west = ios.root.create_entity(
    file, ifc_class='IfcWindow', name='West window', predefined_type='WINDOW'
)
window_west.PartitioningType = single_window_params['partition_type']
window_west_representation = ios.geometry.add_window_representation(
    file, context=body, **single_window_params
)
ios.spatial.assign_container(file, product=window_west, relating_structure=storey)
ios.geometry.edit_object_placement(
    file, product=window_west, matrix=placement_matrix(
        [
            -storey_size.x / 2 - wall_thickness,
            (
                + single_window_params['overall_width'] - wall_thickness / 3
                + triple_window_params['lining_properties']['LiningOffset']
                + triple_window_params['lining_properties']['LiningDepth']
            ),
            window_base_height
        ], x_local=[0., -1., 0.]
    )
)
ios.geometry.assign_representation(
    file, product=window_west, representation=window_west_representation
)
ios.void.add_filling(file, opening=west_opening_copy, element=window_west)

window_left = ios.root.create_entity(
    file, ifc_class='IfcWindow', name='Left Window', predefined_type='WINDOW'
)
window_left.PartitioningType = triple_window_params['partition_type']
window_left_representation = ios.geometry.add_window_representation(
    file, context=body, **triple_window_params
)
ios.spatial.assign_container(file, product=window_left, relating_structure=storey)
ios.geometry.edit_object_placement(
    file, product=window_left, matrix=placement_matrix(
        [
            (
                -storey_size.x / 2 - wall_thickness
                + single_window_params['lining_properties']['LiningOffset']
            ),
            -wall_thickness / 3,
            window_base_height]
    )
)
ios.geometry.assign_representation(
    file, product=window_left, representation=window_left_representation
)
ios.void.add_filling(file, opening=west_opening, element=window_left)

window_style = ios.style.add_style(file)
ios.style.add_surface_style(
    file, style=window_style, ifc_class='IfcSurfaceStyleShading', attributes=window_colour.info
)
ios.style.assign_representation_styles( #q10: Will it be possible to assign different styles to the panel and the lining?
    file, shape_representation=window_right_representation, styles=[window_style]
)
ios.style.assign_representation_styles(
    file, shape_representation=window_west_representation, styles=[window_style]
)
ios.style.assign_representation_styles(
    file, shape_representation=window_left_representation, styles=[window_style]
);


file.write('1.ifc')