#!/usr/bin/env python

import actionlib_msgs.msg
import franka_gripper.msg
import geometry_msgs.msg
import controller_manager_msgs.srv
import moveit_commander
import actionlib
import rospy
import sys


class Context:
    def __init__(self, move_group_name):
        rospy.loginfo('Waiting for move_group/status')
        rospy.wait_for_message('move_group/status',
                               actionlib_msgs.msg.GoalStatusArray)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)

        rospy.loginfo('Waiting for franka_gripper/grasp')
        self.gripper_grasp = actionlib.SimpleActionClient(
            'franka_gripper/grasp',
            franka_gripper.msg.GraspAction)
        self.gripper_grasp.wait_for_server()

        rospy.loginfo('Waiting for franka_gripper/move')
        self.gripper_move = actionlib.SimpleActionClient(
            'franka_gripper/move',
            franka_gripper.msg.MoveAction)
        self.gripper_move.wait_for_server()

        rospy.loginfo('Waiting for controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy(
            'controller_manager/switch_controller',
            controller_manager_msgs.srv.SwitchController)
        self.switch_controller.wait_for_service()

        rospy.loginfo('Waiting for controller_manager/list_controllers')
        self.list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        self.list_controllers.wait_for_service()

        self.active_controllers = []

    def load_controllers(self, controllers):
        params = controller_manager_msgs.srv.SwitchControllerRequest()
        params.start_controllers = controllers
        params.stop_controllers = self.active_controllers
        params.strictness = params.STRICT
        if not self.switch_controller(params):
            rospy.logerr("Couldn't switch controllers")
            sys.exit(1)
        self.active_controllers = controllers


def moveit_joint(ctx, pose):
    ctx.load_controllers(['position_joint_trajectory_controller'])

    print('Moving to joint pose {}'.format(pose))
    ctx.commander.go(pose, wait=True)
    ctx.commander.stop()


def moveit_cart(ctx, pos, rot):
    ctx.load_controllers(['position_joint_trajectory_controller'])

    print('Moving to Cartesian pose: pos {}, rot {}'.format(pos, rot))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = rot[0]
    pose_goal.orientation.x = rot[1]
    pose_goal.orientation.y = rot[2]
    pose_goal.orientation.z = rot[3]
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]
    ctx.commander.set_pose_target(pose_goal)
    ctx.commander.go(wait=True)
    ctx.commander.stop()
    ctx.commander.clear_pose_targets()


def gripper_move(ctx, width, timeout=1.0,
                 epsilon_inner=0.005, epsilon_outer=0.005,
                 speed=0.1, force=10):
    goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
    rospy.loginfo('Moving gripper:\n{}'.format(goal))
    ctx.gripper_move.send_goal(goal)
    ctx.gripper_move.wait_for_result(rospy.Duration.from_sec(timeout))


def gripper_grasp(ctx, width, timeout=1.0,
                  epsilon_inner=0.005, epsilon_outer=0.005,
                  speed=0.1, force=10):
    epsilon = franka_gripper.msg.GraspEpsilon(inner=epsilon_inner,
                                              outer=epsilon_outer)
    goal = franka_gripper.msg.GraspGoal(width=width,
                                        epsilon=epsilon,
                                        speed=speed,
                                        force=force)
    rospy.loginfo('Grasping:\n{}'.format(goal))
    ctx.gripper_grasp.send_goal(goal)
    ctx.gripper_grasp.wait_for_result(rospy.Duration.from_sec(timeout))


def run_controller(ctx, controller_name, wait=None):
    ctx.load_controllers([controller_name])
    if wait is not None:
        rospy.sleep(rospy.Duration.from_sec(wait))


STEPS = {
  'moveit_cart': moveit_cart,
  'moveit_joint': moveit_joint,
  'gripper_move': gripper_move,
  'gripper_grasp': gripper_grasp,
  'run_controller': run_controller,
}


def create_step(t, params):
    callback = STEPS[t]
    return lambda ctx: callback(ctx, **params)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('demo')

    rospy.loginfo('Waiting for controller_manager/load_controller')
    load_controller = rospy.ServiceProxy(
        'controller_manager/load_controller',
        controller_manager_msgs.srv.LoadController)
    load_controller.wait_for_service()

    for controller_name in rospy.get_param('~controllers'):
        if not load_controller(controller_name):
            rospy.logerr('Could not load {}', controller_name)
            sys.exit(1)
    rospy.loginfo('Loaded controllers')

    ctx = Context('panda_arm')

    config = rospy.get_param('~config')
    steps = [create_step(x['type'], x['params']) for x in config]

    rospy.loginfo('Running steps')
    for step in steps:
        step(ctx)

    rospy.loginfo('Performed all steps, shutting down')
