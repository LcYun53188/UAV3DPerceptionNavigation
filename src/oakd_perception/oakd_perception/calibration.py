"""Convert factory coordinate mappings into ROS parent-to-child transforms."""


def inverse_extrinsics(matrix):
    """Invert DepthAI's destination-from-source mapping, converting cm to m.

    A ROS TF whose parent is the source frame describes the child pose in that
    source frame; DepthAI get*Extrinsics(source, destination) maps points in the
    opposite direction. Rotation transpose and -R^T t are both required.
    """
    rotation = [[float(matrix[c][r]) for c in range(3)] for r in range(3)]
    translation = [-sum(rotation[r][c] * float(matrix[c][3]) for c in range(3))/100
                   for r in range(3)]
    return rotation, translation
