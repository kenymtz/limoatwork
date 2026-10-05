from geometry_msgs.msg import PoseStamped


def create_pose(x, y, z, ox, oy, oz, ow):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose


pose_dict = {
    "WS01": create_pose(
        2.11425637, 0.91716720, 0.0,
        0.0, 0.0, -0.99998351, 0.00574281
    ),

    "WS02": create_pose(
        1.73104715, 2.14988173, 0.0,
        0.0, 0.0, 0.68924448, 0.72452885
    ),

    "WS03": create_pose(
        0.75441845, 2.31251507, 0.0,
        0.0, 0.0, 0.99981031, 0.01947694
    ),

    "WS04": create_pose(
        0.34645380, 0.78562654, 0.0,
        0.0, 0.0, 0.99996502, 0.00836465
    ),

    "WS05": create_pose(
        -0.14263704, 0.02349760, 0.0,
        0.0, 0.0, -0.02040766, 0.99979174
    ),
}
