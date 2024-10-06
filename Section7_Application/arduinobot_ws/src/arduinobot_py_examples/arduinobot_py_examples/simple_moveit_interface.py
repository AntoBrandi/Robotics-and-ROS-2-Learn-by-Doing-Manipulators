import rclpy
import numpy as np
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def move_robot():
    arduinobot = MoveItPy(node_name="moveit_py")
    arduinobot_arm = arduinobot.get_planning_component("arm")
    arduinobot_gripper = arduinobot.get_planning_component("gripper")
    
    arm_state = RobotState(arduinobot.get_robot_model())
    gripper_state = RobotState(arduinobot.get_robot_model())
    
    arm_state.set_joint_group_positions("arm", np.array([1.57, 0.0, 0.0]))
    gripper_state.set_joint_group_positions("gripper", np.array([-0.7, 0.7]))
    
    arduinobot_arm.set_start_state_to_current_state()
    arduinobot_gripper.set_start_state_to_current_state()
    
    arduinobot_arm.set_goal_state(robot_state=arm_state)
    arduinobot_gripper.set_goal_state(robot_state=gripper_state)
    arm_plan_result = arduinobot_arm.plan()
    gripper_plan_result = arduinobot_gripper.plan()
    
    if arm_plan_result and gripper_plan_result:
        get_logger("rclpy").info("Planner SUCCEED, moving the arm and the gripper")
        arduinobot.execute(arm_plan_result.trajectory, controllers=[])
        arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("One or more planners failed!")

def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()

if __name__ == "__main__":
    main()