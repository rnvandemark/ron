from math import sqrt, cos, sin, pi as PI

from panda3d.core import Geom, GeomNode, GeomTriangles, GeomVertexData, \
    GeomVertexFormat, GeomVertexWriter, NodePath

BRIDGE_NODE_RGB = (0.67, 0.67, 0.67)
ROBOT_NODE_RGB = (0.2, 0.2, 0.2)

def build_node(node, lights, length, width, height):
    node.setScale(length, width, height)
    node.set_texture_off(1)
    for l in lights:
        node.setLight(l)
    return node

def generate_box(loader, lights, length, width, height):
    return build_node(
        loader.loadModel("models/box"),
        lights,
        length,
        width,
        height
    )

def generate_bridge_node(loader, lights, radius):
    bn = build_node(
        loader.loadModel("models/misc/sphere"),
        lights,
        radius,
        radius,
        radius
    )
    bn.setColor(*BRIDGE_NODE_RGB)
    return bn

def generate_robot_node(lights, major_radius, minor_radius_factor, precision):
    r = minor_radius_factor
    h = sqrt(1 - (r**2))
    rads_step = 2 * PI / precision

    data = GeomVertexData("data", GeomVertexFormat.getV3(), Geom.UHStatic)
    vertices = GeomVertexWriter(data, "vertex")
    triangles = GeomTriangles(Geom.UHStatic)

    # Add the center vertex for the two sides
    vertices.addData3f(0, 0, -h)
    vertices.addData3f(0, 0, +h)

    # Add all the vertices
    for i in range(precision):
        rx, ry = tuple(f(i*rads_step) for f in (cos, sin))
        vertices.addData3f(r*rx, r*ry, -h)
        vertices.addData3f(1*rx, 1*ry,  0)
        vertices.addData3f(r*rx, r*ry, +h)

    # Create the primitive for the bottom side
    triangles.addVertices(0, 2, (3*(precision-1))+2)
    for i in range(1,precision):
        triangles.addVertices(0, (i*3)+2, ((i-1)*3)+2)
    triangles.closePrimitive()

    # Create the primitive for the top side
    triangles.addVertices(1, (3*(precision-1))+4, 4)
    for i in range(1,precision):
        triangles.addVertices(1, ((i-1)*3)+4, (i*3)+4)
    triangles.closePrimitive()

    # Helper function to build a square primitive
    def buildColumnSquares(sl, sr):
        triangles.addVertices(sl+1, sl+0, sr+0)
        triangles.addVertices(sl+1, sr+0, sr+1)
        triangles.closePrimitive()
        triangles.addVertices(sl+2, sl+1, sr+1)
        triangles.addVertices(sl+2, sr+1, sr+2)
        triangles.closePrimitive()

    # Create the primitives for the top and bottom rings
    buildColumnSquares((3*(precision-1))+2, 2)
    for i in range(1,precision):
        buildColumnSquares(((i-1)*3)+2, (i*3)+2)

    geom = Geom(data)
    geom.addPrimitive(triangles)
    node = GeomNode("RobotNode")
    node.addGeom(geom)

    return build_node(
        NodePath(node),
        lights,
        major_radius,
        major_radius,
        major_radius
    )
