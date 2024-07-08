import ifcopenshell
import uuid


def create_guid():
    return ifcopenshell.guid.compress(uuid.uuid1().hex)


def create_wall(ifc_file, context, corner_point, length, height, thickness, name):
    p1 = [float(corner_point[0]), float(corner_point[1]), float(corner_point[2])]
    p2 = [float(corner_point[0] + length), float(corner_point[1]), float(corner_point[2])]
    p3 = [float(corner_point[0] + length), float(corner_point[1] + thickness), float(corner_point[2])]
    p4 = [float(corner_point[0]), float(corner_point[1] + thickness), float(corner_point[2])]
    points = [p1, p2, p3, p4]
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


def main(output_file=r'E:\pcd_workspace\recon\dataset\overlapping_walls.ifc'):
    ifc_file = ifcopenshell.file()

    # Create project and site
    project = ifc_file.createIfcProject(GlobalId=create_guid(), Name="Overlapping Walls Project")
    context = ifc_file.createIfcGeometricRepresentationContext(
        ContextIdentifier='Model',
        ContextType='Model',
        CoordinateSpaceDimension=3,
        Precision=0.00001,  # 0.002
        WorldCoordinateSystem=ifc_file.createIfcAxis2Placement3D(ifc_file.createIfcCartesianPoint((0.0, 0.0, 0.0))),
        TrueNorth=None
    )
    site = ifc_file.createIfcSite(GlobalId=create_guid(), Name="Default Site")
    ifc_file.createIfcRelAggregates(GlobalId=create_guid(), RelatingObject=project, RelatedObjects=[site])

    # Para.
    length = 5.0
    height = 3.0
    thickness = 0.2
    wall1_corner = [0.0, 0.0, 0.0]
    wall2_corner = [3.0, 0.0, 0.0]
    wall1 = create_wall(ifc_file, context, wall1_corner, length, height, thickness, "Wall 1")
    wall2 = create_wall(ifc_file, context, wall2_corner, length, height, thickness, "Wall 2")
    ifc_file.createIfcRelContainedInSpatialStructure(
        GlobalId=create_guid(),
        RelatingStructure=site,
        RelatedElements=[wall1, wall2]
    )
    ifc_file.write(output_file)


if __name__ == "__main__":
    main()
