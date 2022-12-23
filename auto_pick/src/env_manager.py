import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetModelState

class Model():
    def __init__(self, name: str = None, pose: Pose = None, sdf_name: str = None) -> None:  
        self.name, self.pose, self.sdf_name = name, pose, sdf_name

    def sucess(self, object_pose: Pose, gripper_pose: Pose) -> bool:
        """
        Check whether grasping is successful or not

        Returns
        -------
        success : bool
            result of the grasping
        """
        return

class EnvManager():
    def __init__(self) -> None:
        self.permanent_objects = self.get_gazebo_objects()

    def get_gazebo_objects(self) -> list:
        """
        Save a list of objects in the gazebo world

        Returns
        -------
        obj_list : 1xN : obj : `list`
            object list composed with Models
        """
        rospy.wait_for_service('/gazebo/get_world_properties')
        world_state_client = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties)
        obj_names, obj_list = world_state_client.call().model_names, []
        for name in obj_names:
            pose = self.get_gazebo_pose(name)
            obj_list.append(Model(name, pose))

        return obj_list

    def sync_with_gazebo(self) -> list:
        self.permanent_objects = self.get_gazebo_objects()

    @staticmethod
    def spawn_object(name: str, pose: Pose, sdf_name: str):
        """
        Spawn an object in the gazebo world

        Parameters
        ----------
        name : string
            name of the object in the gazebo world
        pose : obj : `Pose`
            pose of the object when spawning
        sdf_name : string
            sdf file of the object we spawn in the gazebo world
        -------
        """
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name=name,
        model_xml=open(f'../../models/{sdf_name}/model.sdf', 'r').read(),
        robot_namespace='/foo', initial_pose=pose, reference_frame='world')

    @staticmethod
    def delete_object(name: str):
        """
        Delete an object in the gazebo world

        Parameters
        ----------
        name : string
            name of the object in the gazebo world
        """
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model_client.call(model_name=name)

    @staticmethod
    def get_gazebo_pose(name: str) -> Pose:
        """
        Retrieve an object pose from the gazebo world

        Parameters
        ----------
        name : string
            name of the object in the gazebo world

        Returns
        -------
        `Pose` : current pose of the object in the gazebo world
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state_client = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        state = model_state_client.call(model_name=name, relative_entity_name='map')

        return state.pose
