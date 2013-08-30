#!/usr/bin/env python
import roslib; roslib.load_manifest('logistic_scenario')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from prace_primitives.msg import MoveLinAction, MoveLinGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

# protected region customHeaders on begin #
from cob_object_detection_msgs.msg import DetectObjectsAction, DetectObjectsGoal
# protected region customHeaders end #



class simple_pnp_impl:
  
  def __init__(self):
    self.cob_marker_goal = DetectObjectsGoal()
    self.MovePTP_goal = MoveLinGoal()
    self.MoveStart_goal = MoveLinGoal()
    self.MoveJointSpace_goal = FollowJointTrajectoryGoal()
    self.MoveBaseHome_goal = MoveBaseActionGoal()
    self.MoveBaseShelf_goal = MoveBaseActionGoal()
   
    # protected region initCode on begin #
    print rospy.get_param('/simple_pnp/MovePTPGoal')
    genpy.message.fill_message_args(self.MovePTP_goal, rospy.get_param('/simple_pnp/MovePTPGoal'))
    print rospy.get_param('/simple_pnp/MoveStartGoal')
    genpy.message.fill_message_args(self.MoveStart_goal, rospy.get_param('/simple_pnp/MoveStartGoal'))
    #print rospy.get_param('/simple_pnp/MoveBaseHomeGoal')
    #genpy.message.fill_message_args(self.MoveBaseHome_goal, {rospy.get_param('/simple_pnp/MoveBaseHomeGoal')})
    #print rospy.get_param('/simple_pnp/MoveBaseShelfGoal')
    #genpy.message.fill_message_args(self.MoveBaseShelf_goal, {rospy.get_param('/simple_pnp/MoveBaseShelfGoal')})
    #print rospy.get_param('/simple_pnp/CobMarkerGoal')
    #genpy.message.fill_message_args(self.cob_marker_goal, rospy.get_param('/simple_pnp/CobMarkerGoal'))
    self.cob_marker_goal.object_name.data = "AUB"
    #genpy.message.fill_message_args(self.MoveJointSpace_goal, rospy.get_param('simple_pnp/MoveJointSpace'))
    # protected region initCode end #
    pass
  
  def test_cb(self, a, b):
    print "TUTUTUUTUTUUTTUTUT"
    
  def configure(self):
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback'], output_keys = ['action_feedback'])
    sm0.register_transition_cb(self.test_cb)
    sis = smach_ros.IntrospectionServer('simple_pnp', sm0, '/simple_pnp_sm')
    sis.start()
    with sm0:
      smach.StateMachine.add('MoveStart', smach_ros.SimpleActionState('MovePTP', MoveLinAction, self.MoveStart_goal), {
        "succeeded":"DetectMarker",
      })
      #smach.StateMachine.add('MoveBaseHome', smach_ros.SimpleActionState('/move_base', MoveBaseAction, self.MoveBaseHome_goal), {
      #  "succeeded":"DetectMarker",
      #})
      #smach.StateMachine.add('MoveBaseShelf', smach_ros.SimpleActionState('/move_base', MoveBaseAction, self.MoveBaseShelf_goal), {
      #  "succeeded":"DetectMarker",
      #})
      smach.StateMachine.add('DetectMarker', smach_ros.SimpleActionState('/cob_marker/object_detection', DetectObjectsAction, self.cob_marker_goal), {
        "succeeded":"MovePTP",
      })
      smach.StateMachine.add('MovePTP', smach_ros.SimpleActionState('MovePTP', MoveLinAction, self.MovePTP_goal), {
        "succeeded":"succeeded",
      })
    
    # Execute
    outcome = sm0.execute()
  
    # protected region configureCode on begin #
    # protected region configureCode end #
    pass
  
  def update(self):
    # protected region updateCode on begin #
    # protected region updateCode end #
    pass
    
  

class simple_pnp:
  def __init__(self):
    self.impl = simple_pnp_impl()

  
    
  def run(self):
    self.impl.update()

if __name__ == "__main__":
  try:
    rospy.init_node('simple_pnp')
    n = simple_pnp()
    n.impl.configure()
    while not rospy.is_shutdown():
      n.run()
      
  except rospy.ROSInterruptException:
    print "Exit"



