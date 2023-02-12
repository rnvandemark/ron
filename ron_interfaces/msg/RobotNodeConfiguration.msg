# The static configuration data for a single robot node. A robot node has this
# general shape, where the 'T' shapes are the four wheels, and the top and
# bottom face translations are as identical in magnitude as possible but ascii
# drawing made that difficult to show:
#
#     __⟂_____⟂__       <- minor radius
#    /           \                         ▲
#   /             \                        | top face translation
#  /               \                       ▼
# /                 \   <- major radius   ---
# \                 /                      ▲
#  \               /                       |  bottom face translation
#   \             /                        |
#    \           /                         ▼
#     ‾‾T‾‾‾‾‾T‾‾       <- minor radius

# The unique ID of this robot node
uint8 id

# The radius of this robot node around its center (the widest part)
float64 major_radius

# The radius of this robot node around its bottom and top faces
float64 minor_radius

# The straight-line translation from the center of this robot node to the
# surface of the bottom face (excluding the wheels) (this is a negative number)
float64 bottom_face_translation

# The straight-line translation from the center of this robot node to the
# surface of the top face (excluding the wheels) (this is a positive number)
float64 top_face_translation
