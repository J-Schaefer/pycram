from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode, Arms
from pycram.process_module import with_real_robot
from pycram.robot_plans import ParkArmsActionDescription
#from pycram.designators.action_designator import ParkArmsActionDescription
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Robot  # Adjust if needed for your pycram version

@with_real_robot
def park_tiago_arms():

    BulletWorld(mode=WorldMode.DIRECT)

    print("[INFO] Creating Tiago object...")
    tiago = Object("tiago", Robot, "tiago_dual.urdf")

    print("[INFO] Parking both arms of Tiago...")
    ParkArmsActionDescription([Arms.BOTH]).resolve().perform()
    print("[INFO] Arms successfully parked!")


if __name__ == "__main__":
    park_tiago_arms()
