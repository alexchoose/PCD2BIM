import ifcopenshell.util.element
model = ifcopenshell.open(r"D:\Desktop\AC20-FZK-Haus.ifc")
wall = model.by_type("IfcWall")[0]
wall_type = ifcopenshell.util.element.get_type(wall)
print(f"The wall type of {wall.Name} is {wall_type.Name}")