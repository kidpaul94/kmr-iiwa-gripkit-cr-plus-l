import rospy
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetModelState

class Spawn_manager():
    def get_gazebo_objects(self):
        rospy.wait_for_service('/gazebo/get_world_properties')
        world_state_client = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties)
        obj_names = world_state_client.call().model_names
        obj_list = []
        for name in obj_names:
            pose = self.get_gazebo_pose(name)
            obj_list.append(Model(name, pose))
        return obj_list

    def spawn_object(self):
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name='object_test',
        model_xml=open('../../models/rotary_arm/model.sdf', 'r').read(),
        robot_namespace='/foo', initial_pose=Pose(position=Point(0,-0.7,0.685), orientation=(Quaternion(0,0,0,1))), reference_frame='world')

    def delete_object(self, obj):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model_client.call(model_name=obj.name)

    def get_gazebo_pose(name):
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state_client = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        state = model_state_client.call(
            model_name=name,
            relative_entity_name='map'
        )
        return state.pose


def main():
    rospy.init_node('insert_object',log_level=rospy.INFO)
    Spawn_manager().spawn_object()

if __name__ == "__main__":
    main()
