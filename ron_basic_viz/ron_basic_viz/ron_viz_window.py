from direct.showbase.ShowBase import ShowBase

from .primitive_generators import generate_box, generate_bridge_node, \
    generate_robot_node

class RonVizWindow(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        # Add a basic floor
        self.scene_floor = generate_box(self.loader, 20.0, 20.0, 0.001)
        self.scene_floor.setColor(0.6, 0.6, 0.6)
        self.scene_floor.setPos(-10.0, -10.0, 0.0)
        self.scene_floor.reparentTo(self.render)

        # Add a test bridge node
        self.bn = generate_bridge_node(self.loader, 0.125)
        self.bn.setPos(0.5, 1.0, 0.125)
        self.bn.reparentTo(self.render)

        # Add a test robot node
        self.rn = generate_robot_node(0.1, 0.67, 12)
        self.rn.setPos(2.0, 2.0, 0.1)
        self.rn.reparentTo(self.render)
