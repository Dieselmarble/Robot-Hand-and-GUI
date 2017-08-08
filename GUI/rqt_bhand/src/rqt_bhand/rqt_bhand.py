#!/usr/bin/env python
from __future__ import division
import os
import rospkg
import time
import math
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, Signal
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtWidgets import QMessageBox
from python_qt_binding.QtGui import QKeySequence, QPixmap, QStandardItemModel, QStandardItem
from rqt_gui_py.plugin import Plugin
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from bhand_controller.srv import Actions, StartController, TorqueEnable
from bhand_controller.msg import Service, CustomHand, MotorState, MotorStateList
from sensor_msgs.msg import JointState
from rospy.exceptions import ROSException


#angles
MAX_VELOCITY = 1000 # max is 1023 this is for security
MAX_TORQUE = 1000
class BHandGUI(Plugin):

        def __init__(self, context):
		super(BHandGUI, self).__init__(context)

		self.setObjectName('BHandGUI')

		self.read_ros_params()
		self._publisher = None

		self._widget = QWidget()

		# variable to store the sensor data when receives it

		self._joint_data = MotorStateList()
		self._tact_data = None
		# Saves the desired value
		self.desired_ref = JointState()

		rp = rospkg.RosPack()
		#Variable inits
		# DESIRED POSITION

		self.finger1_spread = 0.0
		self.finger2_spread = 0.0
		self.finger3_spread = 0.0
		# DESIRED VELOCITY

		self.finger1_spread_vel = 0.0
		self.finger2_spread_vel = 0.0
		self.finger3_spread_vel = 0.0
		self.finger1_eff = 0.0
		self.finger2_eff = 0.0
		self.finger3_eff = 0.0

		self.max_vel = MAX_VELOCITY
		self.max_torque = MAX_TORQUE
		self.vel_factor = self.max_vel/100.0 # For the slider
  		self.eff_factor = self.max_torque/99
		self.finger_factor = 1
		self.red_string = "background-color: rgb(255,0,0)"
		self.orange_string = "background-color: rgb(255,128,0)"
		self.yellow_string = "background-color: rgb(255,255,0)"
		self.green_string = "background-color: rgb(128,255,0)"
		self.black_string = "background-color: rgb(0,0,0)"
		self.state_string = " "


		# UI
		ui_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'Bhand.ui')
		loadUi(ui_file, self._widget)
		self._widget.setObjectName('BHandGUI')

		pixmap_red_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'red.png')
		pixmap_green_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'green.png')

                self._pixmap_red = QPixmap(pixmap_red_file)
		self._pixmap_green = QPixmap(pixmap_green_file)
		self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_red) # Shows agvs_controller comm state

		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

		# Adds this widget to the context
		context.add_widget(self._widget)

		# Try to connect to the topic
		self._joint_states_topic = '/motor_states/dxl_tty1'
		self._tact_topic = '/pressure'
                #%self.bhand_node_name
		self._command_topic = '/command'#%self.bhand_node_name

		# Intermediate structure to save the read positions, vels and loads
		# Initializes joints data for receiving the read values
		self.joint_state_pointer = {}
                self._joint_data.motor_states=[MotorState(), MotorState(), MotorState()]
		for i in range(0,3):
			self.joint_state_pointer[i] = {'motor': self.motor_ids[i], 'joint': self.joint_names[i], 'values': [0, 0, 0, 0, 0, 0, 0, False]}
		
			#print self.joint_state_pointer[i]['finger']                
                	self._joint_data.motor_states[i].id=i+1
		
               		self._joint_data.motor_states[i].goal=0
			self._joint_data.motor_states[i].position=0
			self._joint_data.motor_states[i].error=0
			self._joint_data.motor_states[i].speed=0
			self._joint_data.motor_states[i].load=0
			self._joint_data.motor_states[i].voltage=0
			self._joint_data.motor_states[i].temperature=0
			self._joint_data.motor_states[i].moving=False
			#print self._joint_data.motor_states[i].id

                        # Initializes joints data for receiving the read values

		# SUBSCRIPTIONS

		try:
			self._joint_subscriber = rospy.Subscriber(self._joint_states_topic, MotorStateList , self._receive_joints_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)

		try:
			self._tact_subscriber = rospy.Subscriber(self._tact_topic, CustomHand, self._receive_tact_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)

		# PUBLICATIONS
		try:
			self._publisher_command = rospy.Publisher(self._command_topic, JointState, queue_size=10)
		except ROSException, e:
			rospy.logerr('BHandGUI: Error creating publisher for topic %s (%s)'%(self._command_topic, e))

		# SERVICES
		try:
			self._service_actions = rospy.ServiceProxy('/actions', Actions)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting service (%s)'%e)
		try:
			self._service_start = rospy.ServiceProxy('/start_controller', StartController)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting service (%s)'%e)
		try:
			self._service_enable = rospy.ServiceProxy('/Torque_enable', TorqueEnable)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting service (%s)'%e)


		''' fixed fingers'''
		self.fixed_fingers = 0

		# HANDLERS
		# Adds handlers to 'press button' event
		self._widget.pushButtonStart.clicked.connect(self.start_button_pressed)
		self._widget.pushButton_stop.clicked.connect(self.stop)
		self._widget.pushButton_grab.clicked.connect(self.grab_button_pressed)
		self._widget.pushButton_open.clicked.connect(self.open_button_pressed)
		self._widget.pushButton_squeeze.clicked.connect(self.squeeze_button_pressed)
		self._widget.pushButton_point.clicked.connect(self.point_button_pressed)
		self._widget.pushButton_disable.clicked.connect(self.disable_button_pressed)
		self._widget.pushButton_enable.clicked.connect(self.enable_button_pressed)
		self._widget.horizontalSlider_3.valueChanged.connect(self.slider__changed)
		self._widget.horizontalSlider_2.valueChanged.connect(self.slider_2_changed)
		self._widget.horizontalSlider_1.valueChanged.connect(self.slider_1_changed)
                self._widget.horizontalSlider_v_f1.valueChanged.connect(self.slider_v_f1_changed)
		self._widget.horizontalSlider_v_f2.valueChanged.connect(self.slider_v_f2_changed)
		self._widget.horizontalSlider_v_f3.valueChanged.connect(self.slider_v_f3_changed)
		self._widget.horizontalSlider_4.valueChanged.connect(self.horizontalSlider_4_changed)
		self._widget.horizontalSlider_5.valueChanged.connect(self.horizontalSlider_5_changed)
		self._widget.horizontalSlider_6.valueChanged.connect(self.horizontalSlider_6_changed)			
		self._init_timers()

        def read_ros_params(self):
		'''
			Read ROS params from server
		'''
		_name = rospy.get_name()

		self.bhand_node_name = rospy.get_param('%s/bhand_node_name'%_name, 'rqt_gui')
		# Reads the configuration of the joints ids
		#self.motor_ids  = rospy.get_param('%s/motor_ids'%(self.bhand_node_name), [1, 2, 3])
		#self.joint_names  = rospy.get_param('%s/joint_names'%(self.bhand_node_name), ['joint_1', 'joint_2', 'joint_3'])
		#rospy.loginfo('%s::read_ros_params: bhand_node_name = %s'%(_name, self.bhand_node_name))
		#rospy.loginfo('%s::read_ros_params: motor_ids = %s'%(_name, self.motor_ids))
		#rospy.loginfo('%s::read_ros_params: joint_names = %s'%(_name, self.joint_names))
		
		self.motor_ids  = [1,2,3]
		self.joint_names  = ['joint_1', 'joint_2', 'joint_3']
		rospy.loginfo('%s::read_ros_params: bhand_node_name = %s'%(_name, self.bhand_node_name))
		rospy.loginfo('%s::read_ros_params: motor_ids = %s'%(_name, self.motor_ids))
		rospy.loginfo('%s::read_ros_params: joint_names = %s'%(_name, self.joint_names))

	# inits the timers used to control the connection ...self._widget.lineEdit_f1_read_pos.setText(
        def _init_timers(self):
		self._topic_connected = False
		self._topic_joint_states_connected = False
		self._topic_timer = time.time()
		self._topic_joint_states_timer = time.time()
		self._topic_timeout_connection = 5 # seconds
		self._timer = QBasicTimer()
		self._timer.start(1, self)
		self._timer_commands = QTimer()
		#self.connect(self._timer_commands, SIGNAL("timeout()"),  self.timeout_command_timer)
	def start_button_pressed(self):
		try:
			self._service_start() 
		except ValueError, e:
			rospy.logerr('BHandGUI::start_controller: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::start_controller: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Servicio no disponible")
		'''
			Handles the press button event to call initialization service
		'''
	def disable_button_pressed(self):	
		rospy.wait_for_service('add_two_ints')
		try:
			self.service_enable()
			resp1=service_enable(True)
			return resp1
		except ValueError, e:
			rospy.logerr('BHandGUI::torque_disable: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::torque_disable: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Servicio no disponible")
	def enable_button_pressed(self):
		rospy.wait_for_service('add_two_ints')
		try:
			self.service_enable()
			resp1=service_enable(True)
			return resp1
		except ValueError, e:
			rospy.logerr('BHandGUI::torque_enable: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::torque_enable: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Servicio no disponible")
		
	def grab_button_pressed(self):
		self.send_action(Service.GRAB_GRASP)

	def open_button_pressed(self):
		self.send_action(Service.OPEN_GRASP)

	def squeeze_button_pressed(self):
		self.send_action(Service.SQUEEZE_GRASP)

       	def point_button_pressed(self):
		self.send_action(Service.POINT_GRASP)
	
	def send_action(self, action):
		'''
			Calls the service to set the control mode of the hand
			@param action: Action number (defined in msgs/Service.msg)
			@type action: int
		'''
		try:
			ret = self._service_actions(action)
		except ValueError, e:
			rospy.logerr('BHandGUI::send_action: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::send_action: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Servicio no disponible")

	def slider_2_changed(self):
		self.finger2_spread = 6200 - self._widget.horizontalSlider_2.value() * self.finger_factor
		#if self.fixed_fingers == 1:
		if self._widget.checkBox_position.isChecked():
			self.finger1_spread = self.finger3_spread = self.finger2_spread
			self._widget.horizontalSlider_3.setSliderPosition(self._widget.horizontalSlider_2.value())
			self._widget.horizontalSlider_1.setSliderPosition(self._widget.horizontalSlider_2.value())
		self.send_position_command()

	def slider__changed(self):
		self.finger3_spread = 5400 - self._widget.horizontalSlider_3.value() * self.finger_factor
		#if self.fixed_fingers == 1:
		if self._widget.checkBox_position.isChecked():
			self.finger1_spread = self.finger2_spread = self.finger3_spread
			self._widget.horizontalSlider_2.setSliderPosition(self._widget.horizontalSlider_3.value())
			self._widget.horizontalSlider_1.setSliderPosition(self._widget.horizontalSlider_3.value())
		self.send_position_command()

	def slider_1_changed(self):
		self.finger1_spread = 5400 - self._widget.horizontalSlider_1.value() * self.finger_factor
		#if self.fixed_fingers == 1:
		if self._widget.checkBox_position.isChecked():
			self.finger3_spread = self.finger2_spread = self.finger1_spread
			self._widget.horizontalSlider_2.setSliderPosition(self._widget.horizontalSlider_1.value())
			self._widget.horizontalSlider_3.setSliderPosition(self._widget.horizontalSlider_1.value())
		self.send_position_command()

	def slider_v_f1_changed(self):
		'''
			Handler for slider v_f1
		'''
		self.finger1_spread_vel = self._widget.horizontalSlider_v_f1.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f2.setSliderPosition(self._widget.horizontalSlider_v_f1.value())
			self._widget.horizontalSlider_v_f3.setSliderPosition(self._widget.horizontalSlider_v_f1.value())
		self.send_velocity_command()


	def slider_v_f2_changed(self):
		'''
			Handler for slider v_f2
		'''
		self.finger2_spread_vel = self._widget.horizontalSlider_v_f2.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f1.setSliderPosition(self._widget.horizontalSlider_v_f2.value())
			self._widget.horizontalSlider_v_f3.setSliderPosition(self._widget.horizontalSlider_v_f2.value())
		self.send_velocity_command()

	def slider_v_f3_changed(self):
		'''
			Handler for slider v_f3
		'''
		self.finger3_spread_vel = self._widget.horizontalSlider_v_f3.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f2.setSliderPosition(self._widget.horizontalSlider_v_f3.value())
			self._widget.horizontalSlider_v_f1.setSliderPosition(self._widget.horizontalSlider_v_f3.value())
		self.send_velocity_command()

	def horizontalSlider_4_changed(self):
		self.finger1_eff = self._widget.horizontalSlider_4.value() * self.eff_factor
		self.send_effort_command()

	def horizontalSlider_5_changed(self):
		self.finger2_eff = self._widget.horizontalSlider_5.value() * self.eff_factor
		self.send_effort_command()

	def horizontalSlider_6_changed(self):
		self.finger3_eff = self._widget.horizontalSlider_6.value() * self.eff_factor	
		self.send_effort_command()

	# Handles the messages from the agvs controller

	def _receive_joints_data(self,msg):
		self._joint_data = msg
		self._topic_joint_states_timer = time.time()
		for i in range(0,3):
			for j in range(0,3):
				if self.joint_state_pointer[j]['motor']== msg.motor_states[i].id:
					self.joint_state_pointer[j]['values'] = [msg.motor_states[i].goal, msg.motor_states[i].position, msg.motor_states[i].error, msg.motor_states[i].speed, msg.motor_states[i].load, msg.motor_states[i].voltage, msg.motor_states[i].temperature, msg.motor_states[i].moving]
		if not self._topic_joint_states_connected:
			rospy.loginfo('dynamixel: connection stablished with %s'%self._joint_states_topic)
			self._topic_joint_states_connected = True


	def _receive_tact_data(self, msg):

		if self._tact_data is None:
			self._tact_data = CustomHand()
		self._tact_data = msg

	def radio_button_position_clicked(self, value):
		'''
			Handles the click of this radio button
		'''
		self.set_control_mode('POSITION')
		self._timer_commands.stop()
		self.stop()


	def radio_button_velocity_clicked(self, value):
		'''
			Handles the click of this radio button
		'''
		self.set_control_mode('VELOCITY')
		self._timer_commands.start(dynamixel_VELOCITY_COMMANDS_FREQ)
	def send_position_command(self):
		'''
			Sends a command in position mode
		'''
		self.desired_ref.header.stamp = rospy.Time.now()
		self.desired_ref.header.frame_id='DMT_hand'
		self.desired_ref.name = [self.joint_state_pointer[0]['joint'], self.joint_state_pointer[1]['joint'], self.joint_state_pointer[2]['joint']]
		self.desired_ref.position = [self.finger1_spread, self.finger2_spread, self.finger3_spread]
		self.desired_ref.velocity = [self.finger1_spread_vel, self.finger2_spread_vel, self.finger3_spread_vel]
		self.desired_ref.effort = [0.0, 0.0, 0.0]

                '''Publish Position Command'''
		self._publisher_command.publish(self.desired_ref)

	def send_velocity_command(self):
		'''
			Sends a command to the dynamixel
		'''
		self.desired_ref.header.stamp = rospy.Time.now()
		self.desired_ref.header.frame_id='DMT_hand'
		self.desired_ref.name = [self.joint_state_pointer[0]['joint'], self.joint_state_pointer[1]['joint'], self.joint_state_pointer[2]['joint']]
		self.desired_ref.position = [self.finger1_spread, self.finger2_spread, self.finger3_spread]
		self.desired_ref.velocity = [self.finger1_spread_vel, self.finger2_spread_vel, self.finger3_spread_vel]
		self.desired_ref.effort = [0.0, 0.0, 0.0]
		self._publisher_command.publish(self.desired_ref)

	def send_effort_command(self):
		self.desired_ref.header.stamp = rospy.Time.now()
		self.desired_ref.header.frame_id='DMT_hand'
		self.desired_ref.name = [self.joint_state_pointer[0]['joint'], self.joint_state_pointer[1]['joint'], self.joint_state_pointer[2]['joint']]
		self.desired_ref.position = [self.finger1_spread, self.finger2_spread, self.finger3_spread]
		self.desired_ref.velocity = [0, 0, 0]
		self.desired_ref.effort = [self.finger1_eff, self.finger2_eff, self.finger3_eff]
		self._publisher_command.publish(self.desired_ref)
	
	def timeout_command_timer(self):
		'''
			Handles every timeout triggered by the Qtimer for sending commands
		'''
		self.send_velocity_command()


	def stop(self):
		self._widget.horizontalSlider_v_f1.setValue(0)
		self._widget.horizontalSlider_v_f2.setValue(0)
		self._widget.horizontalSlider_v_f3.setValue(0)


	def shutdown_plugin(self):
		self._timer_commands.stop()
		self._timer.stop()
		self._joint_subscriber.unregister()
		self._tact_subscriber.unregister()
		self._publisher_command.unregister()
		self._service_actions.close()

	# Method executed periodically
	# Updates the graphical qt components

	def timerEvent(self, e):

		#Temperatures
		self._widget.lineEdit_5.setText(str(round(self.joint_state_pointer[0]['values'][6],3)))
		self._widget.lineEdit_6.setText(str(round(self.joint_state_pointer[1]['values'][6],3)))
		self._widget.lineEdit_7.setText(str(round(self.joint_state_pointer[2]['values'][6],3)))
		# Desired Control Positions
		self._widget.lineEdit_f1_des_pos.setText(str(round(self.finger1_spread,1)))
		self._widget.lineEdit_f2_des_pos.setText(str(round(self.finger2_spread,1)))
		self._widget.lineEdit_f3_des_pos.setText(str(round(self.finger3_spread,1)))
		# Read Control Positions
		self._widget.lineEdit_f1_read_pos.setText(str(round(self.joint_state_pointer[0]['values'][1],3)))
		self._widget.lineEdit_f2_read_pos.setText(str(round(self.joint_state_pointer[1]['values'][1],3)))
		self._widget.lineEdit_f3_read_pos.setText(str(round(self.joint_state_pointer[2]['values'][1],3)))
		# Desired velocities
		self._widget.lineEdit_f1_des_vel.setText(str(round(self.finger1_spread_vel,3)))
		self._widget.lineEdit_f2_des_vel.setText(str(round(self.finger2_spread_vel,3)))
		self._widget.lineEdit_f3_des_vel.setText(str(round(self.finger3_spread_vel,3)))
		#Desired Torque
		self._widget.lineEdit_f1_des_eff.setText(str(round(self.finger1_eff,3)))	
		self._widget.lineEdit_f2_des_eff.setText(str(round(self.finger2_eff,3)))
		self._widget.lineEdit_f3_des_eff.setText(str(round(self.finger3_eff,3)))	
		#Current Torque
		self._widget.lineEdit_9.setText(str(round(self.joint_state_pointer[0]['values'][4],3)))
		self._widget.lineEdit_8.setText(str(round(self.joint_state_pointer[1]['values'][4],3)))
		self._widget.lineEdit_2.setText(str(round(self.joint_state_pointer[2]['values'][4],3)))
		#Speed
		self._widget.lineEdit_f1_read_v_2.setText(str(round(self.joint_state_pointer[0]['values'][3],3)))
		self._widget.lineEdit_f2_read_v_2.setText(str(round(self.joint_state_pointer[1]['values'][3],3)))
		self._widget.lineEdit_f3_read_v_2.setText(str(round(self.joint_state_pointer[2]['values'][3],3)))
		#Voltage
		self._widget.lineEdit_11.setText(str(round(self.joint_state_pointer[0]['values'][5],3)))
		self._widget.lineEdit_10.setText(str(round(self.joint_state_pointer[1]['values'][5],3)))
		self._widget.lineEdit_3.setText(str(round(self.joint_state_pointer[2]['values'][5],3)))
		#Bool_Moving
		if self.joint_state_pointer[0]['values'][7]== False:		
			self._widget.qlabel_moving_connection_2.setPixmap(self._pixmap_red)
		else:
			self._widget.qlabel_moving_connection_2.setPixmap(self._pixmap_green)

		if self.joint_state_pointer[1]['values'][7]== False:		
			self._widget.qlabel_moving_connection_3.setPixmap(self._pixmap_red)
		else:
			self._widget.qlabel_moving_connection_3.setPixmap(self._pixmap_green)

		if self.joint_state_pointer[2]['values'][7]== False:		
			self._widget.qlabel_moving_connection_4.setPixmap(self._pixmap_red)
		else:
			self._widget.qlabel_moving_connection_4.setPixmap(self._pixmap_green)

		#Tactile sensors

		#define rgb colors
		self.red = 255
		self.green = 255
		self.blue = 0
		color_string = "background-color: rgb(" + str(self.red) + "," + str(self.green) + "," + str(self.blue) + ")"
		if self._tact_data is not None:

			#Finger 1
			for i in range(0,3):
					lcd_string = "lcdNumber" + str(13 + i)
					value = round(self._tact_data.finger1[(i)], 1)
					getattr(self._widget,lcd_string).display(value)
					if 0.0 <= value and value < 300.0:
						color_string = self.green_string
					elif 300.0 <= value and value < 500.0:
						color_string = self.yellow_string
					elif 500.0 <= value and value < 700.0:
						color_string = self.orange_string
					elif 700.0 <= value and value <= 900.0:
						color_string = self.red_string
					else:
						color_string = self.black_string
					getattr(self._widget,lcd_string).setStyleSheet(color_string)

			#Finger 2
			for i in range(0,3):
					lcd_string = "lcdNumber" + str(8 + i)
					value = round(self._tact_data.finger2[i], 1)
					getattr(self._widget,lcd_string).display(value)
					if 0.0 <= value and value < 300.0:
						color_string = self.green_string
					elif 300.0 <= value and value < 500.0:
						color_string = self.yellow_string
					elif 500.0 <= value and value < 700.0:
						color_string = self.orange_string
					elif 900.0 <= value and value <= 900.0:
						color_string = self.red_string
					else:
						color_string = self.black_string
					getattr(self._widget,lcd_string).setStyleSheet(color_string)

			#Finger 3
			for i in range(0,3):
					lcd_string = "lcdNumber" + str(16 + i)
					value = round(self._tact_data.finger3[i], 1)
					getattr(self._widget,lcd_string).display(value)
					if 0.0 <= value and value < 300.0:
						color_string = self.green_string
					elif 300.0 <= value and value < 500.0:
						color_string = self.yellow_string
					elif 500.0 <= value and value < 700.0:
						color_string = self.orange_string
					elif 700.0 <= value and value <= 900.0:
						color_string = self.red_string
					else:
						color_string = self.black_string
					getattr(self._widget,lcd_string).setStyleSheet(color_string)

			#Palm
			for i in range(0,2):
				lcd_string = "lcdNumber" + str(1 + i) + "_6"
				value = round(self._tact_data.palm[i], 1)
				getattr(self._widget,lcd_string).display(value)
				if 0.0 <= value and value < 2.5:
					color_string = self.green_string
				elif 2.5 <= value and value < 5.0:
					color_string = self.yellow_string
				elif 5.0 <= value and value < 7.5:
					color_string = self.orange_string
				elif 7.5 <= value and value <= 16.0:
					color_string = self.red_string
				else:
					color_string = self.black_string
				getattr(self._widget,lcd_string).setStyleSheet(color_string)
		# Checks the ROS connection
		t = time.time()
		if self._topic_joint_states_connected and (t - self._topic_joint_states_timer >= self._topic_timeout_connection):
			self._topic_joint_states_connected = False
			rospy.logerr('BHandGUI: error in communication with %s'%self._joint_states_topic)
			self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_red)
		if self._topic_joint_states_connected:
			self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_green)
