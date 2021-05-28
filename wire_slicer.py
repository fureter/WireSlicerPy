"""
    Modules to handle differnent functionality revolving around CNC Hotwire cutting and generating slices from 3D
    test_geometry for template creation.

    Goal: Streamline the process of manufacturing complex 3D objects using CNC hotwire cutters, or by generating
    printable templates.

    Functions:
    1. Parse DXF and various ascii formats to read and export spline data to represent cross sections
    2. Manipulate closed splines:
        -Break closed shapes into sub areas
        -Offset splines to create hollow shapes
        -Transform splines in 3d space
    3. Group closed spline sets to create a cut profile on two parallel offset planes
    4. Optimally Place close spline groups onto a 3d plane to minimize uncovered space
    5. Generate an movement path for a CNC Hotwire cutter that is optimized based on different contraint inputs
    6. Convert spline group placements on a plane to G-Code to move two independent CNC Gantries
    7. Parse STL files representing closed 3D bodies to generate spline groups

    Dependencies:
    -Numpy
    -Scipy
    -matplotlib
    -trimesh
    -shapely
    -pyglet

    Optional:
    -coverage
    -pylint

"""