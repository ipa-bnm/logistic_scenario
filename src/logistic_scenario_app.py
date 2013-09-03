#!/usr/bin/env python
import roslib; roslib.load_manifest('logistic_scenario')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cob_object_detection_msgs.msg import DetectObjectsAction, DetectObjectsGoal
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from prace_primitives.msg import MoveLinAction, MoveLinGoal


# protected region customHeaders on begin #
# protected region customHeaders end #



class logistic_scenario_app_impl:
	
	def	__init__(self):
		self.MoveHomePTP_goal = MoveLinGoal()
		genpy.message.fill_message_args(self.MoveHomePTP_goal, [rospy.get_param('/logistic_scenario_app/MoveHomePTP')])
		self.MoveBaseHome_goal = MoveBaseGoal()
		genpy.message.fill_message_args(self.MoveBaseHome_goal, [rospy.get_param('/logistic_scenario_app/MoveBaseHome')])
		self.MoveBaseShelf_goal = MoveBaseGoal()
		genpy.message.fill_message_args(self.MoveBaseShelf_goal, [rospy.get_param('/logistic_scenario_app/MoveBaseShelf')])
		self.DetectObjects_goal = DetectObjectsGoal()
		genpy.message.fill_message_args(self.DetectObjects_goal, [rospy.get_param('/logistic_scenario_app/DetectObjects')])
		self.MoveObjectPTP_goal = MoveLinGoal()
		genpy.message.fill_message_args(self.MoveObjectPTP_goal, [rospy.get_param('/logistic_scenario_app/MoveObjectPTP')])
		self.MoveLinGrasp_goal = MoveLinGoal()
		genpy.message.fill_message_args(self.MoveLinGrasp_goal, [rospy.get_param('/logistic_scenario_app/MoveLinGrasp')])
		self.MoveLinGraspUp_goal = MoveLinGoal()
		genpy.message.fill_message_args(self.MoveLinGraspUp_goal, [rospy.get_param('/logistic_scenario_app/MoveLinGraspUp')])
		self.MoveLinGraspBack_goal = MoveLinGoal()
		genpy.message.fill_message_args(self.MoveLinGraspBack_goal, [rospy.get_param('/logistic_scenario_app/MoveLinGraspBack')])
	
		# protected region initCode on begin #
        # protected region initCode end #
		pass
	
	def	configure(self):
		sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback'], output_keys = ['action_feedback'])
		sis = smach_ros.IntrospectionServer('logistic_scenario_app', sm0, '/logistic_scenario_app_sm')
		sis.start()
		with sm0:
			smach.StateMachine.add('MoveHomePTP', smach_ros.SimpleActionState('/MovePTP', MoveLinAction, self.MoveHomePTP_goal), {
				"succeeded":"MoveBaseHome",
			})
			smach.StateMachine.add('MoveBaseHome', smach_ros.SimpleActionState('/move_base', MoveBaseAction, self.MoveBaseHome_goal), {
				"succeeded":"MoveBaseShelf",
			})
			smach.StateMachine.add('MoveBaseShelf', smach_ros.SimpleActionState('/move_base', MoveBaseAction, self.MoveBaseShelf_goal), {
				"succeeded":"DetectObjects",
			})
			smach.StateMachine.add('DetectObjects', smach_ros.SimpleActionState('/cob_marker/object_detection', DetectObjectsAction, self.DetectObjects_goal), {
				"succeeded":"MoveObjectPTP",
			})
			smach.StateMachine.add('MoveObjectPTP', smach_ros.SimpleActionState('/MovePTP', MoveLinAction, self.MoveObjectPTP_goal), {
				"succeeded":"MoveLinGrasp",
			})
			smach.StateMachine.add('MoveLinGrasp', smach_ros.SimpleActionState('/MoveLin', MoveLinAction, self.MoveLinGrasp_goal), {
				"succeeded":"MoveLinGraspUp",
			})
			smach.StateMachine.add('MoveLinGraspUp', smach_ros.SimpleActionState('/MoveLin', MoveLinAction, self.MoveLinGraspUp_goal), {
				"succeeded":"MoveLinGraspBack",
			})
			smach.StateMachine.add('MoveLinGraspBack', smach_ros.SimpleActionState('/MoveLin', MoveLinAction, self.MoveLinGraspBack_goal), {
				"succeeded":"succeeded",
			})
	

		# Execute
		outcome = sm0.execute()
	
		# protected region configureCode on begin #
        # protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
        # protected region updateCode end #
		pass
		
	

class logistic_scenario_app:
	def __init__(self):
		self.impl = logistic_scenario_app_impl()

	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('logistic_scenario_app')
		n = logistic_scenario_app()
		n.impl.configure()
		while not rospy.is_shutdown():
			n.run()
			
	except rospy.ROSInterruptException:
		print "Exit"



