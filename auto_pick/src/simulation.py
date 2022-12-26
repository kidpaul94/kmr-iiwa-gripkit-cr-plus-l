import time
import rospy

from utils import Conversion
from robot_manager import Move_Robot
from env_manager import Model, EnvManager

def main():
    rospy.init_node("auto_pick", anonymous=True)
    """
    Currently testing
    """
    EM, conv = EnvManager(), Conversion()
    pose = conv.list2pose(sixd=[0., -0.65, 0.69, 0, 0, -180.])
    EM.spawn_object(name='object_1', pose=pose, sdf_name='rotary_arm')
    EM.sync_with_gazebo()
    Move_Robot().cartesian_space(waypoints=[pose])
    
    # rospy.spin()

if __name__ == "__main__":
    main()
