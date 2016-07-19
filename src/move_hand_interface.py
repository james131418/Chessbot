import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import (
PoseStamped,
Pose,
Point,
Quaternion,
)
import tf
from std_msgs.msg import String
import baxter_interface
from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
)


class HandMover():
	def __init__(self):
		## First initialize moveit_commander and rospy.
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_piece_interface',
				anonymous=True)

		## Instantiate a RobotCommander object.  This object is an interface to
		## the robot as a whole.
		self.robot = moveit_commander.RobotCommander()

		## Instantiate a PlanningSceneInterface object.  This object is an interface
		## to the world surrounding the robot.
		self.scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a MoveGroupCommander object.  This object is an interface
		## to one group of joints.  In this case the group is the joints in the left
		## arm.  This interface can be used to plan and execute motions on the left
		## arm.
		self.left_group = moveit_commander.MoveGroupCommander("left_arm")
		self.right_group = moveit_commander.MoveGroupCommander("right_arm")
		#self.listener = tf.TransformListener()
		#self.listener.waitForTransform("/base", 'left_gripper', rospy.Time(0), rospy.Duration(3.0));
		#self._limb = baxter_interface.Limb('left')
		self._left_gripper = baxter_interface.Gripper('left')
		self._right_gripper = baxter_interface.Gripper('right')
		self.gripper_close()

	def move_hand_interface(self, x, y, z, arm='left'):
		#(trans, rot) = self.listener.lookupTransform('/base', 'left_gripper', rospy.Time(0))
		left_rotation = geometry_msgs.msg.Quaternion()
		left_rotation.x = -0.0249590815779
		left_rotation.y = 0.999649402929
		left_rotation.z = 0.00737916180073
		left_rotation.w = 0.00486450832011
		right_rotation = geometry_msgs.msg.Quaternion()
		right_rotation.x = 0.00606654
		right_rotation.y = 1
		right_rotation.z = -0.00474318
		right_rotation.w = 0.00300499
#		print "rotation"
#		print rotation
		#quaternion = (rot[0], rot[1],rot[2],rot[3])
		#print tf.transformations.euler_from_quaternion(quaternion)
		pose_target = geometry_msgs.msg.Pose()
		if arm == 'left':
			pose_target.orientation.x = left_rotation.x
			pose_target.orientation.y = left_rotation.y
			pose_target.orientation.z = left_rotation.z
			pose_target.orientation.w = left_rotation.w
		if arm == 'right':
			pose_target.orientation.x = right_rotation.x
			pose_target.orientation.y = right_rotation.y
			pose_target.orientation.z = right_rotation.z
			pose_target.orientation.w = right_rotation.w
		pose_target.position.x = x
		pose_target.position.y = y
		pose_target.position.z = z
		
		if arm == 'left':
			self.left_group.set_pose_target(pose_target)
			print '============Waiting for moving left arm...'
			print 'Target position: %s, %s, %s' % (x, y, z)
			self.left_group.plan()
			self.left_group.go(wait=True)
			self.left_group.clear_pose_targets()
		elif arm == 'right':
			self.right_group.set_pose_target(pose_target)
			print '============Waiting for moving right arm...'
			print 'Target position: %s, %s, %s' % (x, y, z)
			self.right_group.plan()
			self.right_group.go(wait=True)
			self.right_group.clear_pose_targets()
		else:
			print '============Error. No arm specified========='
		#return self.listener.lookupTransform('/base', 'left_gripper', rospy.Time(0))

	def gripper_open(self, block = False, timeout = 5.0):
		rospy.sleep(1.0)
		self._left_gripper.command_position(position=72.0, block=block, timeout=timeout)
		self._right_gripper.command_position(position=72.0, block=block, timeout=timeout)
		rospy.sleep(0.5)

	def gripper_close(self):
		rospy.sleep(1.5)
		self._left_gripper.set_velocity(2)
		self._right_gripper.set_velocity(2)
		self._left_gripper.close()
		self._right_gripper.close()
		rospy.sleep(1.0)

	def shutdown(self):
		moveit_commander.roscpp_shutdown()

