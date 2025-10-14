from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.robot_description import RobotDescription
import pycram.tasktree
from pycram.datastructures.enums import Arms, ObjectType
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
import anytree
import pycram.failures
import time


from pycrap.ontologies import Milk, Cereal, Robot, Kitchen
world = BulletWorld(WorldMode.GUI)

kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl", pose=Pose([1.3, 0.7, 0.95]))
milk_desig = ObjectDesignatorDescription(names=["milk"])
cereal_desig = ObjectDesignatorDescription(names=["cereal"])


kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

robot = input("enter robot name: /n")
if robot == "p":
    robot = Object("pr2", Robot, "pr2.urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
else :
    robot = Object("tiago", Robot, "tiago_dual.urdf", pose=Pose([0.5, 1, 0],[0, 0,0 ,1]))
    robot_desig = ObjectDesignatorDescription(names=["tiago"]).resolve()


cereal_target = cereal_desig.resolve()
if cereal_target is None:
    print("Error: Cereal object could not be resolved!")
else:
    print("Cereal object resolved!")


@pycram.tasktree.with_tree
def plan():
    with simulated_robot:
        ParkArmsActionPerformable(Arms.BOTH).perform()
        print("Step 1: Arms parked")
        time.sleep(1)

        MoveTorsoAction([TorsoState.MID]).resolve().perform()
        print("Step 2: Torso moved")
        time.sleep(1)

        print("Resolving cereal object...")
        cereal_target = cereal_desig.resolve()
        if cereal_target is None:
            print("ERROR: Cereal object could not be resolved!")
            return

        print("Cereal object resolved:", cereal_target)

        print("Resolving pickup location...")


        pickup_pose = CostmapLocation(target=cereal_desig.resolve(), reachable_for=robot_desig).resolve()
        if pickup_pose is None:
            print("ERROR: Could not resolve pickup pose!")
            return

        print("Pickup pose resolved:", pickup_pose)

        print("Checking reachable arms...")
        if not pickup_pose.reachable_arms:
            print("ERROR: No reachable arm found for picking up the cereal!")
            return

        pickup_arm = pickup_pose.reachable_arms[0]
        print("Reachable arm selected:", pickup_arm)
        time.sleep(1)

        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()
        print("Step 3: Navigated to cereal")

        PickUpAction(object_designator_description=cereal_desig, arms=[pickup_arm],
                     grasps=[Grasp.FRONT]).resolve().perform()
        print("Step 4: Picked up the cereal")

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Step 5: Arms parked")

        place_island = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                               cereal_desig.resolve()).resolve()

        place_stand = CostmapLocation(place_island.pose, reachable_for=robot_desig, reachable_arm=pickup_arm).resolve()

        NavigateAction(target_locations=[place_stand.pose]).resolve().perform()
        print("Step 6: Navigated to kitchen island")

        PlaceAction(cereal_desig, target_locations=[place_island.pose], arms=[pickup_arm]).resolve().perform()
        print("Step 7: Placed cereal on island")

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Step 8: Arms parked again")

        ParkArmsActionPerformable(Arms.BOTH).perform()
        print("Step 9: Final arm parking")


plan()