#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
import argparse
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from  baxter_core_msgs.msg import DigitalIOState
from baxter_core_msgs.msg import EndEffectorCommand
from baxter_core_msgs.msg import NavigatorState





def init():
	global left

        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
	def offset_position(gripper, offset):
                if gripper.type() != 'electric':
                    capability_warning(gripper, 'command_position')
                    return
                current = gripper.position()
                gripper.command_position(current + offset)
	rs.enable()
def main():
	

        epilog = """
    See help inside the example with the '?' key for key bindings.
       """

        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=main.__doc__,
                                         epilog=epilog)
        parser.parse_args(rospy.myargv()[1:])
     
        print("Initializing node... ")
        rospy.init_node('button_state', anonymous=True)
	init()

	#left cuff
	def sol_cuff_alt_sag(state):
	    	if state.state == 1:
			rospy.loginfo("sol cuff alt sag basıldı")
			left.command_position(0.0)
	def sol_cuff_ust(state):
		if state.state == 1:
			left.command_position(50.0)
			rospy.loginfo("sol cuff ust basıldı")
	def sol_cuff_alt_sol(state):
		if state.state == 1:
			rospy.loginfo("sol cuff alt sol basıldı")
			left.command_position(100.0)

	#right cuff
	def sag_cuff_alt_sag(state):
		if state.state == 1:
			rospy.loginfo("sag cuff alt sag basıldı")
	def sag_cuff_alt_sol(state):
		if state.state == 1:
			rospy.loginfo("sag cuff alt sol basıldı")
	def sag_cuff_ust(state):
		if state.state == 1:
			rospy.loginfo("sag cuff ust basıldı")

	#left arm
	def left_arm_buttons(buttons):
		if buttons.buttons == [False, True , False]:
			rospy.loginfo("left arm ust basıldı")
		if buttons.buttons == [True, False , False]:
			rospy.loginfo("left arm OK basıldı")
		if buttons.buttons == [False, False , True]:
			rospy.loginfo("left arm alt basıldı")
	def left_arm_wheel(wheel):
		rospy.loginfo(wheel.wheel)
		mapvalue = (wheel.wheel * 100)/255
		left.command_position(mapvalue)
		
	#left torso
	def left_torso_buttons(buttons):
		if buttons.buttons == [False, True , False]:
			rospy.loginfo("left torso ust basıldı")
		if buttons.buttons == [True, False , False]:
			rospy.loginfo("left torso OK basıldı")
		if buttons.buttons == [False, False , True]:
			rospy.loginfo("left torso alt basıldı")
	def left_torso_wheel(wheel):
		rospy.loginfo(wheel.wheel)

		
	#right arm
	def right_arm_buttons(buttons):
		if buttons.buttons == [False, True , False]:
			rospy.loginfo("right arm ust basıldı")
		if buttons.buttons == [True, False , False]:
			rospy.loginfo("right arm OK basıldı")
		if buttons.buttons == [False, False , True]:
			rospy.loginfo("right arm alt basıldı")
	def right_arm_wheel(wheel):
		rospy.loginfo(wheel.wheel)

	#right torso
	def right_torso_buttons(buttons):
		if buttons.buttons == [False, True , False]:
			rospy.loginfo("right torso ust basıldı")
		if buttons.buttons == [True, False , False]:
			rospy.loginfo("right torso OK basıldı")
		if buttons.buttons == [False, False , True]:
			rospy.loginfo("right torso alt basıldı")
	def right_torso_wheel(wheel):
		rospy.loginfo(wheel.wheel)		

    	rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState , sol_cuff_alt_sag)
	rospy.Subscriber("/robot/digital_io/left_upper_button/state", DigitalIOState , sol_cuff_alt_sol)
	rospy.Subscriber("/robot/digital_io/left_lower_cuff/state", DigitalIOState , sol_cuff_ust)
    	rospy.Subscriber("/robot/digital_io/right_lower_button/state", DigitalIOState , sag_cuff_alt_sag)
	rospy.Subscriber("/robot/digital_io/right_upper_button/state", DigitalIOState , sag_cuff_alt_sol)
	rospy.Subscriber("/robot/digital_io/right_lower_cuff/state", DigitalIOState , sag_cuff_ust)

	rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, left_arm_buttons)
	rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, left_arm_wheel)
	rospy.Subscriber("/robot/navigators/torso_left_navigator/state", NavigatorState, left_torso_buttons)
	rospy.Subscriber("/robot/navigators/torso_left_navigator/state", NavigatorState, left_torso_wheel)

	rospy.Subscriber("/robot/navigators/right_navigator/state", NavigatorState, right_arm_buttons)
	rospy.Subscriber("/robot/navigators/right_navigator/state", NavigatorState, right_arm_wheel)
	rospy.Subscriber("/robot/navigators/torso_right_navigator/state", NavigatorState, right_torso_buttons)
	rospy.Subscriber("/robot/navigators/torso_rightt_navigator/state", NavigatorState, right_torso_wheel)
	
	rospy.spin()

if __name__ == '__main__':
	main()
