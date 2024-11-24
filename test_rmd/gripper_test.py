from pyRobotiqGripper import RobotiqGripper
from time import sleep

gripper = RobotiqGripper()

gripper.open()

sleep(1)

gripper.close()

sleep(1)

gripper.open()