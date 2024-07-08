import ifcopenshell
import ifcopenshell.api
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Any
from nptyping import NDArray, Shape, Float
import os
import uuid

@dataclass
class IfcOpenShellPythonAPI:
    module_stack: List[str] = field(default_factory=list)

    def __getattr__(self, module: str) -> Optional[Any]:
        if module == 'shape'or module.startswith('_'):
            return
        self.module_stack.append(module)
        return self

    def __call__(self, *args, **kwargs) -> Any:
        try:
            result = ifcopenshell.api.run('.'.join(self.module_stack), *args, **kwargs)
        except Exception as err:
            raise err
        finally:
            self.reset()
        return result

    def reset(self) -> None:
        self.module_stack = []


def placement_matrix(
        placement: list[float, float, float],
        x_local: Optional[list[float, float, float]] = None,
        z_local: Optional[list[float, float, float]] = None,
        scale: float = 1.
) -> NDArray[Shape['4, 4'], Float]:
    """Utility function to obtain the placement matrix for a given position, axis coordinates and scale factor"""

    arr_placement: NDArray[Shape['3'], Float] = np.array(placement)
    arr_x_local: NDArray[Shape['3'], Float] = np.array([1., 0., 0.]) if x_local is None else np.array(x_local)
    arr_z_local: NDArray[Shape['3'], Float] = np.array([0., 0., 1.]) if z_local is None else np.array(z_local)
    arr_y_local = np.cross(arr_z_local, arr_x_local).reshape(-1, 1)  # X (RefDirection) and Z (Axis) are enough
    arr_x_local, arr_z_local = arr_x_local.reshape(-1, 1), arr_z_local.reshape(-1, 1)
    scale_row = np.array([0., 0., 0., scale])
    arr = np.hstack([arr_x_local, arr_y_local, arr_z_local])
    arr = np.hstack([arr, arr_placement.reshape(-1, 1)])
    return np.vstack([arr, scale_row])



@dataclass
class ColourRGB:
    red: float
    green: float
    blue: float
    name: Optional[str] = None
    transparency: float = 0.

    @property
    def info(self):
        surface_colour = {key.capitalize(): value for key, value in asdict(self).items() if key != 'transparency'}
        return {'SurfaceColour': surface_colour, 'Transparency': self.transparency}


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
    return float(wall_height), float(wall_thickness), float(wall_length),wall_vp, wall_name

def find_aabb_min_max(points):
    # Convert the list of points to a numpy array for easier manipulation
    points_array = np.array(points)

    # Find the min and max for each dimension
    min_point = np.min(points_array, axis=0)
    max_point = np.max(points_array, axis=0)

    return min_point.tolist(), max_point.tolist()

def calculate_parameters(min_point, max_point, classes):
    # Unpack min and max points
    min_x, min_y, min_z = min_point
    max_x, max_y, max_z = max_point
    # Calculate dimensions
    if (max_x - min_x) > (max_y - min_y):
        width = max_x - min_x
        thickness = max_y - min_y
        orientation = [1, 0, 0]
        placement = [
            min_x ,
            (min_y + max_y) / 2,
            min_z
        ]
    else:
        thickness = max_x - min_x
        width = max_y - min_y
        orientation = [0, 1, 0]
        placement = [
            (min_x + max_x) / 2,
             min_y ,
             min_z
        ]

    height = max_z - min_z

    # Calculate placement (center of the AABB)


    return width, height, thickness, placement, orientation



def main():
    #/****************定义ios文件****************/
    # Initialize API
    ios = IfcOpenShellPythonAPI()
    # Define parameters
    project_name = 'IFC Building Project'
    site_name, building_name, storey_name = 'Site', 'Building', 'Storey'
    # Create file
    file = ios.project.create_file(version='IFC4')
    project = ios.root.create_entity(file, ifc_class='IfcProject', name=project_name)
    ios.project.assign_declaration(file, definition=project, relating_context=project)
    # Create contexts
    ctx = ios.context.add_context(file, context_type='Model')
    body = ios.context.add_context(file, parent=ctx, context_identifier='Body', target_view='MODEL_VIEW')
    # Create site, building, and storey
    site = ios.root.create_entity(file, ifc_class='IfcSite', name=site_name)
    ios.aggregate.assign_object(file, product=site, relating_object=project)
    building = ios.root.create_entity(file, ifc_class='IfcBuilding', name=building_name)
    ios.aggregate.assign_object(file, product=building, relating_object=site)
    storey = ios.root.create_entity(file, ifc_class='IfcBuildingStorey', name=storey_name)
    ios.aggregate.assign_object(file, product=storey, relating_object=building)
    #/****************定义创建元素函数****************/
    def creat_element(data_class, name, width, height, thickness, placement, orientation):
        # /****************定义颜色****************/
        wall_colour = ColourRGB(.75, 0.73, 0.68)
        footing_colour = ColourRGB(.38, 0.4, 0.42)
        window_colour = ColourRGB(.5, 0.4, 0.3, transparency=0.8)
        door_colour = ColourRGB(.8, .8, .8)
        length = width
        if data_class == 'door':
            colour = door_colour
            door = ios.root.create_entity(file, ifc_class='IfcDoor', name=name, predefined_type='DOOR')
            door_representation = ios.geometry.add_door_representation(
                file, context=body, width=width, height=height, thickness=thickness
            )
            ios.spatial.assign_container(file, product=door, relating_structure=storey)
            ios.geometry.edit_object_placement(
                file, product=door, matrix=placement_matrix(placement, x_local=orientation)
            )
            ios.geometry.assign_representation(file, product=door, representation=door_representation)
            door_style = ios.style.add_style(file)
            ios.style.add_surface_style(
                file, style=door_style, ifc_class='IfcSurfaceStyleShading', attributes=colour.info
            )
            ios.style.assign_representation_styles(
                file, shape_representation=door_representation, styles=[door_style]
            )
        elif data_class == 'wall':
            colour = wall_colour
            wall = ios.root.create_entity(file, ifc_class='IfcWall', name=name, predefined_type='SOLIDWALL')
            wall_representation = ios.geometry.add_wall_representation(
                file, context=body, length=length, height=height, thickness=thickness
            )
            ios.spatial.assign_container(file, product=wall, relating_structure=storey)
            ios.geometry.edit_object_placement(
                file, product=wall, matrix=placement_matrix(placement, x_local=orientation)
            )
            ios.geometry.assign_representation(file, product=wall, representation=wall_representation)
            wall_style = ios.style.add_style(file)
            ios.style.add_surface_style(
                file, style=wall_style, ifc_class='IfcSurfaceStyleShading', attributes=colour.info
            )
            ios.style.assign_representation_styles(
                file, shape_representation=wall_representation, styles=[wall_style]
            )
        elif data_class == 'window':
            colour = window_colour
            window = ios.root.create_entity(file, ifc_class='IfcWindow', name=name, predefined_type='WINDOW')
            window_representation = ios.geometry.add_window_representation(
                file, context=body, width=width, height=height, thickness=thickness
            )
            ios.spatial.assign_container(file, product=window, relating_structure=storey)
            ios.geometry.edit_object_placement(
                file, product=window, matrix=placement_matrix(placement, x_local=orientation)
            )
            ios.geometry.assign_representation(file, product=window, representation=window_representation)
            window_style = ios.style.add_style(file)
            ios.style.add_surface_style(
                file, style=window_style, ifc_class='IfcSurfaceStyleShading', attributes=colour.info
            )
            ios.style.assign_representation_styles(
                file, shape_representation=window_representation, styles=[window_style]
            )
        elif data_class == 'floor':
            colour = footing_colour
            footing = ios.root.create_entity(file, ifc_class='IfcFooting', name=name, predefined_type='STRIP_FOOTING')
            footing_representation = ios.geometry.add_wall_representation(
                file, context=body, length=length, height=height, thickness=thickness
            )
            ios.spatial.assign_container(file, product=footing, relating_structure=storey)
            ios.geometry.edit_object_placement(
                file, product=footing, matrix=placement_matrix(placement, x_local=orientation)
            )
            ios.geometry.assign_representation(file, product=footing, representation=footing_representation)
            footing_style = ios.style.add_style(file)
            ios.style.add_surface_style(
                file, style=footing_style, ifc_class='IfcSurfaceStyleShading', attributes=colour.info
            )
            ios.style.assign_representation_styles(
                file, shape_representation=footing_representation, styles=[footing_style]
            )

    # 信息读取
    input_information_paths = r'D:\ctz\code\PC_I\PCD2BIM_paper_lantop\wall_window_door_floor\information_temp'
    for classes in os.listdir(input_information_paths):
        input_class_data_paths = os.path.join(input_information_paths, classes)

        for input_class_data_path in os.listdir(input_class_data_paths):
            input_class_data_path = os.path.join(input_class_data_paths, input_class_data_path)
            height, thickness, length, element_vp, element_name = txt_information(input_class_data_path)
            element_vp_min_point, element_vp_max_point = find_aabb_min_max(element_vp)
            length, _, thickness, placement, orientation = \
                calculate_parameters(element_vp_min_point, element_vp_max_point, classes)
            creat_element(classes, element_name, length, height, thickness, placement, orientation)
            # Save the IFC file
    file.write("output.ifc")
    print("IFC file created successfully!")









# 示例使用
if __name__ == '__main__':
    main()

