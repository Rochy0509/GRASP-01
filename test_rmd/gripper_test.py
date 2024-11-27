from pyRobotiqGripper import RobotiqGripper
from time import sleep

# Initialize the gripper
gripper = RobotiqGripper()

print("Control the gripper by pressing 'o' to open, 'c' to close, or 'q' to quit.")

while True:
    # Get user input
    command = input("Enter command (o: open, c: close, q: quit): ").strip().lower()
    
    if command == 'o':
        gripper.open()
        print("Gripper is opening...")
        sleep(1)  # Optional: wait for the action to complete
    elif command == 'c':
        gripper.close()
        print("Gripper is closing...")
        sleep(1)  # Optional: wait for the action to complete
    elif command == 'q':
        print("Exiting...")
        break
    else:
        print("Invalid command. Please press 'o', 'c', or 'q'.")
