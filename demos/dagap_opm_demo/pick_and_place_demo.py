#!/usr/bin/env python3

from __future__ import print_function

# ROS Imports
import rospy

# DAGAP Imports
from dagap_msgs.srv import *
from dagap_msgs.msg import *
import dagap.utils.tfwrapper as dagap_tf

# PyCRAM Imports
from pycram.process_module import simulated_robot, real_robot
from pycram.designators.action_designator import *  # includes import typing
from pycram.designators.specialized_designators.action.dual_arm_pickup_action import DualArmPickupAction
from pycram.datastructures.enums import Arms, ObjectType, Grasp
from pycram.designators.object_designator import *
from pycram.worlds.bullet_world import BulletWorld, Object, WorldMode
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.designator import ObjectDesignatorDescription
from pycram.datastructures.pose import Pose
from pycram.plan_failures import IKError
from pycram.local_transformer import LocalTransformer

import pycram.external_interfaces.giskard as giskardpy


class PickAndPlaceDemo:

    def __init__(self, use_dual_arm: bool = True,
                 use_opm: bool = True,
                 object_spawning_poses: List[Pose] = None,
                 object_placing_poses: List[Pose] = None):
        self.reference_frame = "iai_kitchen/sink_area_surface"
        dagap_tf.init()  # call tfwrapper init()

        self.use_dual_arm = use_dual_arm  # use dual arm pickup action heuristic
        self.use_opm = use_opm  # use OPM service

        # Set up the bullet world
        self.world = BulletWorld(WorldMode.GUI)
        self.world.set_gravity([0, 0, -9.8])

        self.tfbroadcaster = TFBroadcaster()
        self.local_transformer = LocalTransformer()  # PyCRAM tf transformer

        # Spawn ground plane
        # self.plane = Object(name="floor", obj_type=ObjectType.ENVIRONMENT, path="plane.urdf", world=self.world)
        # plane.set_color([0, 0, 0, 1])

        # Spawn kitchen
        self.kitchen = Object(name="kitchen", obj_type=ObjectType.ENVIRONMENT, path="kitchen.urdf")
        # kitchen.set_color([0.2, 0, 0.4, 0.6])
        self.kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

        sink_area_surface_frame = self.kitchen.get_link_tf_frame("sink_area_surface")
        kitchen_island_surface_frame = self.kitchen.get_link_tf_frame("kitchen_island_surface")

        # Set up object spawning and placing poses, fall back to default if not provided
        if object_spawning_poses is None:
            self.object_spawning_poses: List[Pose] = [
                Pose([0.2, -0.9, 0.1], [0, 0, 1, 0], frame=sink_area_surface_frame),  # breakfast-cereal
                Pose([0.2, -0.35, 0.05], [0, 0, 1, 0], frame=sink_area_surface_frame),  # cup
                Pose([-0.3, 0.5, 0.05], [0, 0, 0, 1], frame=kitchen_island_surface_frame),  # bowl
                Pose([0.15, -0.4, 0.05], [0, 0, 1, 0], frame=sink_area_surface_frame),  # spoon
                Pose([0.07, 0.4, 0.1], [0, 0, 1, 0], frame=sink_area_surface_frame)  # milk
            ]
        else:
            self.object_spawning_poses = object_spawning_poses

        if object_placing_poses is None:
            self.object_placing_poses: List[Pose] = [
                Pose([0.2, -0.20, 0.1], [0, 0, 1, 0], frame=kitchen_island_surface_frame),  # breakfast-cereal
                Pose([-0.10, -0.80, 0.05], [0, 0, 0, 1], frame=kitchen_island_surface_frame),  # cup
                Pose([-0.24, -0.70, 0.05], [0, 0, 0, 1], frame=kitchen_island_surface_frame),  # bowl
                Pose([-0.24, -0.5, 0.05], [0, 0, 0, 1], frame=kitchen_island_surface_frame),  # spoon
                Pose([-0.3, -1.00, 0.1], [0, 0, 1, 0], frame=kitchen_island_surface_frame)  # milk
            ]
        else:
            self.object_placing_poses = object_placing_poses

        # Hint for type of list object_spawning_poses_map
        self.object_spawning_poses_map: List[Pose] = []
        self.object_placing_poses_map: List[Pose] = []

        # Transform poses to map frame for correct spawn pose
        element: Pose  # Hint for type of element
        for element in self.object_spawning_poses:
            self.object_spawning_poses_map.append(
                # dagap_tf.transform_pose(element, 'simulated/map', element.header.frame_id))
                self.local_transformer.transform_pose(element, target_frame="map"))

        # Transform object_placing_poses into map frame
        element: Pose  # Hint for type of element
        for element in self.object_placing_poses:
            self.object_placing_poses_map.append((
                self.local_transformer.transform_pose(element, target_frame="map")
            ))

        self.object_names = [
            "robot",
            "breakfast-cereal",
            "cup",
            "bowl",
            "spoon",
            "milk"
        ]

        self.query_object_list_map: List[OPMObjectQuery] = [
            OPMObjectQuery(Object="robot", object_location=Pose([0, 0, 0])),  # OPM needs robot in first element
            OPMObjectQuery(Object="breakfast-cereal", object_location=self.object_spawning_poses_map[0]),
            OPMObjectQuery(Object="cup", object_location=self.object_spawning_poses_map[1]),
            OPMObjectQuery(Object="bowl", object_location=self.object_spawning_poses_map[2]),
            OPMObjectQuery(Object="spoon", object_location=self.object_spawning_poses_map[3]),
            OPMObjectQuery(Object="milk", object_location=self.object_spawning_poses_map[4])
        ]

        # Spawn breakfast cereal
        # TODO: change self.query_object_list_map to original pose list self.object_spawning_poses_map
        self.breakfast_cereal = Object(self.query_object_list_map[1].Object,
                                       self.query_object_list_map[1].Object,
                                       path="breakfast_cereal.stl",
                                       pose=self.query_object_list_map[1].object_location)
        self.breakfast_cereal_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[1].Object])
        # Spawn cup
        self.cup = Object(self.query_object_list_map[2].Object,
                          self.query_object_list_map[2].Object,
                          path="cup.stl",
                          pose=self.query_object_list_map[2].object_location)
        self.cup_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[2].Object])
        # Spawn bowl
        self.bowl = Object(self.query_object_list_map[3].Object,
                           self.query_object_list_map[3].Object,
                           path="bowl.stl",
                           pose=self.query_object_list_map[3].object_location)
        self.bowl_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[3].Object])
        # Spawn spoon
        self.spoon = Object(self.query_object_list_map[4].Object,
                            self.query_object_list_map[4].Object,
                            path="spoon.stl",
                            pose=self.query_object_list_map[4].object_location)
        self.spoon_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[4].Object])
        # Spawn milk
        self.milk = Object(self.query_object_list_map[5].Object,
                           self.query_object_list_map[5].Object,
                           path="milk.stl",
                           pose=self.query_object_list_map[5].object_location)
        self.milk_desig = ObjectDesignatorDescription(names=[self.query_object_list_map[5].Object])

        # Spawn PR2 robot
        self.pr2 = Object(name="pr2", obj_type=ObjectType.ROBOT, path="pr2.urdf", pose=Pose([0, 0, 0]))
        self.robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

        self.query_object_list_map[1].object_frame =\
            f"simulated/{self.breakfast_cereal.tf_frame}"
        self.query_object_list_map[2].object_frame =\
            f"simulated/{self.cup.tf_frame}"
        self.query_object_list_map[3].object_frame =\
            f"simulated/{self.bowl.tf_frame}"
        self.query_object_list_map[4].object_frame =\
            f"simulated/{self.spoon.tf_frame}"
        self.query_object_list_map[5].object_frame =\
            f"simulated/{self.milk.tf_frame}"

        # giskardpy.sync_worlds()

        # self.world.add_vis_axis(self.bowl.get_pose())
        # self.world.add_vis_axis(self.breakfast_cereal.get_pose())
        # self.world.add_vis_axis(self.cup.get_pose())
        # self.world.add_vis_axis(self.spoon.get_pose())
        # self.world.add_vis_axis(self.milk.get_pose())

        # Test out an example transform to catch exceptions early
        if dagap_tf.lookup_transform(f"SIMULATED/{self.kitchen.get_link_tf_frame('sink_area_surface')}",
                                     f"SIMULATED/{self.bowl.tf_frame}"):
            rospy.loginfo("Test succeeded: Found transform")
        else:
            rospy.logwarn("Test failed: Did not find transform")

    def get_designator_from_name(self, object_name: str) -> ObjectDesignatorDescription:
        if self.object_names[0] == object_name:
            return self.robot_desig
        if self.object_names[1] == object_name:
            return self.breakfast_cereal_desig
        if self.object_names[2] == object_name:
            return self.cup_desig
        if self.object_names[3] == object_name:
            return self.bowl_desig
        if self.object_names[4] == object_name:
            return self.spoon_desig
        if self.object_names[5] == object_name:
            return self.milk_desig

    def get_object_from_name(self, object_name: str) -> Object:
        if self.object_names[0] == object_name:
            return self.pr2
        if self.object_names[1] == object_name:
            return self.breakfast_cereal
        if self.object_names[2] == object_name:
            return self.cup
        if self.object_names[3] == object_name:
            return self.bowl
        if self.object_names[4] == object_name:
            return self.spoon
        if self.object_names[5] == object_name:
            return self.milk

    def get_name_from_frame(self, frame: str) -> str:
        for name in self.object_names:
            if name in frame:
                return name
        rospy.logwarn("[get_name_from_frame]: Could not find name.")
        return ""

    def get_placing_pose_from_name(self, object_name: str) -> Pose:
        if self.object_names[1] == object_name:
            return self.object_placing_poses_map[0]
        if self.object_names[2] == object_name:
            return self.object_placing_poses_map[1]
        if self.object_names[3] == object_name:
            return self.object_placing_poses_map[2]
        if self.object_names[4] == object_name:
            return self.object_placing_poses_map[3]
        if self.object_names[5] == object_name:
            return self.object_placing_poses_map[4]

    def opm_dagap_client(self, reference_frame: str, object_list: [OPMObjectQuery]) -> GetNextOPMObjectResponse:
        rospy.loginfo("Waiting for service.")
        rospy.wait_for_service('dagap_opm_query')
        try:
            rospy.loginfo("Calling dagap_opm_query.")
            call_common_service = rospy.ServiceProxy('dagap_opm_query', GetNextOPMObject)
            srv = GetNextOPMObjectRequest(reference_frame, object_list)
            response = call_common_service(srv)
            rospy.loginfo("Received response.")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def opm_client(self, object_list: [OPMObjectQuery]) -> GetOPMSortedListResponse:
        rospy.loginfo("Waiting for service.")
        rospy.wait_for_service('opm_query')
        try:
            rospy.loginfo("Calling opm_query.")
            call_opm_service = rospy.ServiceProxy('opm_query', GetOPMSortedList)
            srv = GetOPMSortedListRequest(object_list)
            response = call_opm_service(srv)
            rospy.loginfo("Received response.")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def dagap_client(self, task_description: str, object_frame: [str]) -> GetGraspPoseResponse:
        rospy.loginfo("Waiting for service.")
        rospy.wait_for_service('dagap_query')
        try:
            rospy.loginfo("Calling dagap_query.")
            call_dagap_service = rospy.ServiceProxy('dagap_query', GetGraspPose)
            srv = GetGraspPoseRequest(task_description, object_frame)
            response = call_dagap_service(srv)
            rospy.loginfo("Received response.")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def run(self) -> List[str]:
        """
        Run the pick and place demo.
        """

        rospy.loginfo("Running demo.")
        with (simulated_robot):
            # Send request to DAGAP service
            rospy.set_param(param_name='robot_root',
                            param_value="pr2")

            object_list = self.query_object_list_map
            final_object_list = []
            final_placing_poses = []

            while len(object_list) > 1:

                if self.use_opm:
                    # service return frame not the name
                    next_opm_object: GetOPMSortedListResponse = self.opm_client(object_list=object_list)
                    rospy.loginfo(f"OPM returned: {next_opm_object.next_object}")
                else:
                    # if conservative demo take next object in list
                    next_opm_object = GetOPMSortedListResponse(next_object=object_list[1].object_frame)
                    rospy.loginfo(f"Taking next object: {next_opm_object.next_object}")
                    pass

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                MoveTorsoAction([0.33]).resolve().perform()

                next_object_name = self.get_name_from_frame(next_opm_object.next_object)
                next_object_desig: ObjectDesignatorDescription = self.get_designator_from_name(next_object_name)

                # Remove current object from list for next iteration
                element: OPMObjectQuery
                for element in list(object_list):
                    if element.Object == next_object_name:
                        final_object_list.append(f"{element.Object}: [{element.object_location.position.x}, {element.object_location.position.y}, {element.object_location.position.z}] in {element.object_location.frame}")
                        object_list.remove(element)

                pickup_pose = CostmapLocation(target=next_object_desig.resolve(),
                                              reachable_for=self.robot_desig
                                              ).resolve()
                # pickup_arm = pickup_pose.reachable_arms[0]  # allocate an arm to the grasping task

                NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

                try:
                    if self.use_dual_arm:
                        rospy.loginfo(f"Picking up {next_object_name}")
                        # self.world.add_vis_axis(self.pr2.get_link_pose("r_gripper_tool_frame"))
                        # self.world.add_vis_axis(self.pr2.get_link_pose("l_gripper_tool_frame"))
                        first_pickup = DualArmPickupAction(object_designator_description=next_object_desig,
                                                           grasps=[Grasp.FRONT]
                                                           ).resolve()
                        pickup_arm = first_pickup.arm
                        first_pickup.perform()
                    else:  # if conservative demo
                        pickup_arm = pickup_pose.reachable_arms[0]
                        PickUpAction(object_designator_description=next_object_desig,
                                     arms=[pickup_arm],
                                     grasps=[Grasp.FRONT]
                                     ).resolve().perform()
                except IKError:
                    rospy.logwarn("Failed execution with {} hand.".format(pickup_arm))
                    if pickup_arm == "left":
                        pickup_arm = "right"
                        rospy.loginfo(f"Falling back to {pickup_arm} hand.")
                    elif pickup_arm == "right":
                        pickup_arm = "left"
                        rospy.loginfo(f"Falling back to {pickup_arm} hand.")
                    PickUpAction(object_designator_description=next_object_desig, arms=[pickup_arm],
                                 grasps=[Grasp.FRONT]
                                 ).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                # Get placing position on island
                # place_island = SemanticCostmapLocation(urdf_link_name="kitchen_island_surface",
                #                                        part_of=self.kitchen_desig.resolve(),
                #                                        for_object=next_object_desig.resolve()
                #                                        ).resolve()

                # self.world.remove_vis_axis()

                # Visualize coordinate system of kitchen island
                # nullpose = dagap_tf.transform_pose(
                #     pose=Pose(),
                #     target_frame="simulated/map",
                #     source_frame=dagap_tf.get_closest_matching_frame(
                #         self.kitchen.get_link_tf_frame("kitchen_island_surface"))
                # ).pose
                # self.world.add_vis_axis(
                #     Pose(dagap_tf.point_to_list(nullpose.position),
                #          dagap_tf.quaternion_to_list(nullpose.orientation)))
                # self.world.add_vis_axis(kitchen_island_surface_frame)

                next_placing_pose = self.get_placing_pose_from_name(next_object_name)
                final_placing_poses.append(f"{next_object_name}: [{next_placing_pose.position.x}, {next_placing_pose.position.y}, {next_placing_pose.position.z}] in {next_placing_pose.frame}")
                # self.world.add_vis_axis(next_placing_pose)

                # Get position to stand while placing the object
                place_stand = CostmapLocation(target=next_placing_pose,
                                              reachable_for=self.robot_desig,
                                              reachable_arm=pickup_arm
                                              ).resolve()

                NavigateAction(target_locations=[place_stand.pose]).resolve().perform()

                rospy.loginfo(f"Placing {next_object_name} on kitchen island.")
                PlaceAction(object_designator_description=next_object_desig,
                            target_locations=[next_placing_pose],
                            arms=[pickup_arm]
                            ).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()

                # self.world.remove_vis_axis()  # Remove visualizations
            return final_object_list, final_placing_poses

    def cleanup(self):
        self.world.exit()  # Exit the bullet world
