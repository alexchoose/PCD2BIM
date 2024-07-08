import ifcopenshell
import ifcopenshell.api
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Any
from nptyping import NDArray, Shape, Float


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
    placement: List[float],
    x_local: Optional[List[float]] = None,
    z_local: Optional[List[float]] = None,
    scale: float = 1.
) -> NDArray[Shape['4, 4'], Float]:
    arr_placement = np.array(placement).reshape(3, 1)
    arr_x_local = np.array([1., 0., 0.]).reshape(3, 1) if x_local is None else np.array(x_local).reshape(3, 1)
    arr_z_local = np.array([0., 0., 1.]).reshape(3, 1) if z_local is None else np.array(z_local).reshape(3, 1)
    arr_y_local = np.cross(arr_z_local.ravel(), arr_x_local.ravel()).reshape(3, 1)
    scale_row = np.array([0., 0., 0., scale]).reshape(1, 4)
    arr = np.hstack([arr_x_local, arr_y_local, arr_z_local, arr_placement])
    return np.vstack([arr, scale_row])



@dataclass
class ColourRGB:
    red: float
    green: float
    blue: float
    name: Optional[str] = None
    transparency: float = 0.

    @property
    def info(self) -> dict[str, float | dict[str, float | str | None]]:
        surface_colour = {key.capitalize(): value for key, value in asdict(self).items() if key != 'transparency'}
        return {'SurfaceColour': surface_colour, 'Transparency': self.transparency}


# Initialize API
ios = IfcOpenShellPythonAPI()

# Define parameters
project_name = 'IFC Building Project'
site_name, building_name, storey_name = 'Site', 'Building', 'Storey'

#color parameter
wall_colour = ColourRGB(0.75, 0.73, 0.68)
door_colour = ColourRGB(0.8, 0.5, 0.3)
window_colour = ColourRGB(0.6, 0.7, 0.8)

# Wall parameters
wall_thickness = 0.2
wall_height = 3.0
wall_length = 5.0

# Door parameters
door_width = 1.0
door_height = 2.1
door_thickness =0.1
door_placement = [2.5,-door_thickness, 0]
door_orientation = [1, 0, 0]
# Window parameters
window_width = 1.2
window_height = 1.5
window_thickness = 0.1
window_placement = [4,-window_thickness, 1.0]
window_orientation = [1, 0, 0]

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


def create_wall(name, length, thickness, height, placement, colour, orientation):
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



def create_door(name, width, height, thickness, placement, colour, orientation):
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


def create_window(name, width, height, thickness, placement, colour, orientation):
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


# Create walls
create_wall('North Wall', wall_length, wall_thickness, wall_height, [2, 0, 0], wall_colour, [1, 0, 0])
create_wall('South Wall', wall_length, wall_thickness, wall_height, [2, wall_length , 0], wall_colour, [1, 0, 0])
create_wall('East Wall', wall_length, wall_thickness, wall_height, [2+wall_thickness, 0, 0], wall_colour, [0, 1, 0])
create_wall('West Wall', wall_length, wall_thickness, wall_height, [2+wall_length, 0, 0], wall_colour, [0, 1, 0])

# Create doors
create_door('Main Door', door_width, door_height, door_thickness, door_placement, door_colour, door_orientation)


# Create windows
create_window('Living Room Window', window_width, window_height, window_thickness, window_placement, window_colour, window_orientation)


# Save the IFC file
file.write("output.ifc")
print("IFC file created successfully!")
