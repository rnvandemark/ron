from direct.showbase.ShowBase import ShowBase

from .primitive_generators import generate_box

class RonVizWindow(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        # Add a basic floor
        self.scene_floor = generate_box(self.loader, 20.0, 20.0, 0.001)
        self.scene_floor.setColor(0.6, 0.6, 0.6)
        self.scene_floor.setPos(-10.0, -10.0, 0.0)
        self.scene_floor.reparentTo(self.render)
