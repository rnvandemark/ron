def build_node(node, length, width, height):
    node.setScale(length, width, height)
    node.set_texture_off(1)
    return node

def generate_box(loader, length, width, height):
    return build_node(
        loader.loadModel("models/box"),
        length,
        width,
        height
    )
