from pycram.robot_descriptions.tiago_description import tiago_description
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import WorldMode, Arms, TorsoState, Grasp, GripperState
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.pose import Pose, PoseStamped
from pycram.datastructures.grasp import GraspDescription
from pycram.designators.object_designator import BelieveObject
from pycram.designators.location_designator import CostmapLocation, SemanticCostmapLocation
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.process_module import simulated_robot

#None, PoseStamped.from_list([0.5, 1.0, 0.0]))
from pycrap.ontologies import Milk, Cereal, Robot, Kitchen
world = BulletWorld(WorldMode.GUI)

kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", None, PoseStamped.from_list([1.3, 1, 0.9]))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", None, PoseStamped.from_list([1.3, 0.7, 0.95]))
milk_desig = ObjectDesignatorDescription(names=["milk"])
cereal_desig = ObjectDesignatorDescription(names=["cereal"])
place_stand = PoseStamped.from_list([1.3, 0.7, 0.95])

kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

robot = input("enter robot name: ")
if robot == "p":
    robot = Object("pr2", Robot, "pr2.urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
else:
    robot = Object("tiago", Robot, "tiago_dual.urdf", None, PoseStamped.from_list([0.5, 1, 0],[0, 0,0 ,1]))
    robot_desig = ObjectDesignatorDescription(names=["tiago"]).resolve()

cereal_target = cereal_desig.resolve()
if cereal_target is None:
    print("Error: Cereal object could not be resolved!")
else:
    print("Cereal object resolved!")

with simulated_robot:
        ParkArmsAction(arm=Arms.BOTH).perform()
        print("Arm action performed!")
        MoveTorsoAction(torso_state=TorsoState.HIGH).perform()
        print("Torso action performed!")
        pickup_pose = CostmapLocation(target=cereal_desig.resolve(), reachable_for=robot_desig, reachable_arm=Arms.RIGHT).resolve()
        print("Pickup pose resolved!")
        pickup_arm = Arms.RIGHT
        print(f"Pickup arm used: {pickup_arm}")
        NavigateAction(target_location=pickup_pose).perform()
        print("Navigation action performed!")
        PickUpAction(object_designator=cereal_target, arm=pickup_arm, grasp_description=GraspDescription(Grasp.FRONT, None, False)).perform()
        print("PickUpAction is done")

        ParkArmsAction(arm=Arms.BOTH).perform()
        print("Arm action performed!")

        place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                               cereal_desig.resolve()).resolve()

       # place_stand = CostmapLocation(place_island.pose, reachable_for=robot_desig, reachable_arm=pickup_arm).resolve()


        #print("PlaceIsland action performed!")

        #NavigateAction(target_location=place_island.pose).perform()
        #print("Navigation action performed!")
        #PlaceAction(cereal, target_location=place_island, arm=pickup_arm).perform()
        #print("PlaceAction performed!")
        #ParkArmsAction(arm=Arms.BOTH).perform()



        NavigateAction(target_location=PoseStamped.from_list([-2, 1.7, 0])).perform()

        world.add_vis_axis(robot_desig.get_link_pose("gripper_left_grasping_frame"))


        PlaceAction(cereal, target_location=PoseStamped.from_list([-1.227, 2.099, 0.96], [-0.0, 0.0, 0.553, -0.833]), arm=pickup_arm).perform()
        print("PlaceAction performed!")


        ParkArmsAction(arm=Arms.BOTH).perform()