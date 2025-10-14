
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.object_designator import *
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import WorldMode, TorsoState

from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Robot, Milk, Apartment
from pycram.external_interfaces.ik import try_to_reach

world = BulletWorld(WorldMode.GUI)
apartment = Object("apartment", Apartment, "apartment.urdf")

milk = Object("milk", Milk, "milk.stl", pose=Pose([4.3, 2, 1]))
milk_desig = ObjectDesignatorDescription(names=["milk"]).resolve()
# arm = Arms.LEFT
pr2 = Object("pr2", Robot, "pr2.urdf", pose=Pose([1, 2, 0]))
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
# tiago = Object("tiago", Robot, "tiago_dual.urdf", pose=Pose([1, 2, 0]))
# robot_desig = ObjectDesignatorDescription(names=["tiago"]).resolve()

# milk_pose = milk_desig.pose
# offset_distance = 0.5
# target_x = milk_pose.position.x - offset_distance
# target_y = milk_pose.position.y
# target_z = 0.0

# target_orientation = milk_pose.orientation
# target_pose = Pose([target_x, target_y, target_z], target_orientation)

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    pickup_pose = CostmapLocation(target=milk_desig, reachable_for=robot_desig, reachable_arm=Arms.LEFT).resolve()
    NavigateAction([pickup_pose.pose]).resolve().perform()
