import ifcopenshell
import ifcopenshell.api
import ifcopenshell.util.element
import numpy as np
from mathutils import Vector
from dataclasses import dataclass, field, asdict
from enum import Enum, auto
from typing import Callable, Optional, Any
from nptyping import NDArray, Shape, Float


@dataclass
class IfcOpenShellPythonAPI:
    module_stack: list[str] = field(default_factory=list)

    def __getattr__(self, module: str) -> Optional[Callable]:
        if module == 'shape' or module.startswith('_'):
            return
        self.module_stack.append(module)
        return self

    def __call__(self, *args, **kwargs) -> Any:
        try:
            result: Any = ifcopenshell.api.run('.'.join(self.module_stack), *args, **kwargs)
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
    arr_placement: NDArray[Shape['3'], Float] = np.array(placement)
    arr_x_local: NDArray[Shape['3'], Float] = np.array([1., 0., 0.]) if x_local is None else np.array(x_local)
    arr_z_local: NDArray[Shape['3'], Float] = np.array([0., 0., 1.]) if z_local is None else np.array(z_local)
    arr_y_local = np.cross(arr_z_local, arr_x_local).reshape(-1, 1)
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
    def info(self) -> dict[str, float | dict[str, float | str | None]]:
        surface_colour = {key.capitalize(): value for key, value in asdict(self).items() if key != 'transparency'}
        return {'SurfaceColour': surface_colour, 'Transparency': self.transparency}


# Initialize API
ios = IfcOpenShellPythonAPI()

# Define parameters
project_name = 'IFC Window Project'
site_name, building_name, storey_name = 'Site', 'Building', 'Storey'

# Window parameters
window_width = 1.2
window_height = 1.5
window_thickness = 0.2
window_placement = [1.5, 0, 1.0]
window_orientation = [1, 0, 0]

# Colors
window_colour = ColourRGB(0.6, 0.7, 0.8)

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

# Function to create a window
def create_window(name, width, height, thickness, placement, colour, orientation):
    window = ios.root.create_entity(file, ifc_class='IfcWindow', name=name, predefined_type='WINDOW')
    window_representation = ios.geometry.add_window_representation(
        file, context=body, width=width, height=height, thickness=thickness
    )
    print(placement_matrix(placement, x_local=orientation))
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

# Create windows
create_window('Living Room Window', window_width, window_height, window_thickness, window_placement, window_colour, window_orientation)

# Save file
file.write('window_project.ifc')

print('IFC file with windows generated successfully.')
