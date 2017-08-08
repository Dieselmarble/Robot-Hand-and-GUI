#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
__license__ = 'BSD'

from threading import Thread, Lock
import sys
import rospy
import time
from dynamixel_driver.dynamixel_serial_proxy import SerialProxy
from dynamixel_driver.dynamixel_io import DynamixelIO
from dynamixel_msgs.msg import CustomHand
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import JointState
from dynamixel_controllers.srv import StartController, StartControllerResponse, StopController, StopControllerResponse
from dynamixel_controllers.srv import RestartController, RestartControllerResponse
from dynamixel_controllers.srv import Actions, TorqueEnable
#import Adafruit_GPIO.SPI as SPI
#import Adafruit_MCP3008
# Software SPI configuration for ADC1and ADC2
#CLK  = 18
#MISO_1 = 23
#MOSI_1 = 24
#CS_1   = 25
#MISO_2 = 22
#MOSI_2 = 27
#CS_2 = 10
#mcp_1= Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS_1, miso=MISO_1, mosi=MOSI_1)
#mcp_2 = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS_2, miso=MISO_2, mosi=MOSI_2)

class ControllerManager:
    def __init__(self):
        rospy.init_node('dynamixel_controller_manager', anonymous=True)
        rospy.on_shutdown(self.on_shutdown)
        self.waiting_meta_controllers = []
        self.controllers = {}
        self.serial_proxies = {}
        self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1)
        self.start_controller_lock = Lock()
        self.stop_controller_lock = Lock()
        manager_namespace = rospy.get_param('~namespace')
        serial_ports = rospy.get_param('~serial_ports')
        self.bhand_node_name = 'rqt_gui'
        self.finger_names = ['joint_1', 'joint_2', 'joint_3']
        self.motor_ids = [1, 2, 3]
        #joint_state = JointState()
	#joint_state.name=[0, 0, 0]
	#joint_state.position=[0, 0, 0]
	#joint_state.velocity=[0, 0, 0]
	#joint_state.effort=[0, 0, 0]
	data = JointState()
        for port_namespace,port_config in serial_ports.items():
            port_name = port_config['port_name']
            baud_rate = port_config['baud_rate']
            readback_echo = port_config['readback_echo'] if 'readback_echo' in port_config else False
            min_motor_id = port_config['min_motor_id'] if 'min_motor_id' in port_config else 0
            max_motor_id = port_config['max_motor_id'] if 'max_motor_id' in port_config else 253
            update_rate = port_config['update_rate'] if 'update_rate' in port_config else 5
            error_level_temp = 75
            warn_level_temp = 70
            if 'diagnostics' in port_config:
                if 'error_level_temp' in port_config['diagnostics']:
                    error_level_temp = port_config['diagnostics']['error_level_temp']
                if 'warn_level_temp' in port_config['diagnostics']:
                    warn_level_temp = port_config['diagnostics']['warn_level_temp']
            serial_proxy = SerialProxy(port_name,
                                       port_namespace,
                                       baud_rate,
                                       min_motor_id,
                                       max_motor_id,
                                       update_rate,
                                       self.diagnostics_rate,
                                       error_level_temp,
                                       warn_level_temp,
                                       readback_echo)
            serial_proxy.connect()

        rospy.Service('/start_controller' , StartController, self.start_controller)
        rospy.Service('/stop_controller' , StopController, self.stop_controller)
        rospy.Service('/restart_controller' , RestartController, self.restart_controller)
        rospy.Service('/actions', Actions, self.handActions)
	rospy.Service('/torque_enable', TorqueEnable, self.torque_enable)	
        self.serial_proxies[port_namespace] = serial_proxy

        # services for 'meta' controllers, e.g. joint trajectory controller
        rospy.Service('%s/meta/start_controller' % manager_namespace, StartController, self.start_controller)
        rospy.Service('%s/meta/stop_controller' % manager_namespace, StopController, self.stop_controller)
        rospy.Service('%s/meta/restart_controller' % manager_namespace, RestartController, self.restart_controller)
        rospy.Service('%s/%s/meta/actions' % (manager_namespace, port_namespace), Actions, self.handActions)
        #Publishers
        self.diagnostics_pub = rospy.Publisher('/pressure', CustomHand, queue_size=1)
        if self.diagnostics_rate > 0: Thread(target=self.diagnostics_processor).start()
        #Subscribers
        self._command_topic = '/command'#%self.bhand_node_name
        self._subscriber_command = rospy.Subscriber(self._command_topic, JointState, self.receive_joints_data)

        while not rospy.is_shutdown(): 
		self.receive_joints_data(data)
		#self.position_control()
		#self.speed_control()
		#self.torque_control()

    def on_shutdown(self):
        for serial_proxy in self.serial_proxies.values():
            serial_proxy.disconnect()
    '''def control_loop(self):
	#self.read_sensor()
	self.receive_joints_data(data)
	self.position_control()
	self.speed_control()
	self.torque_control()'''

    def diagnostics_processor(self):
        diag_msg = DiagnosticArray()
        rate = rospy.Rate(self.diagnostics_rate)
        while not rospy.is_shutdown():
            diag_msg.status
            diag_msg.header.stamp = rospy.Time.now()
            for controller in self.controllers.values():
                try:
                    joint_state = controller.joint_state
                    temps = joint_state.motor_temps
                    max_temp = max(temps)
                    status = DiagnosticStatus()
                    status.name = 'Joint Controller (%DMT_HAND)'
                    status.hardware_id = 'Robotis Dynamixel %s on port %s' % (str(joint_state.motor_ids), controller.port_namespace)
                    status.values.append(KeyValue('Goal', str(joint_state.goal_pos)))
                    status.values.append(KeyValue('Position', str(joint_state.current_pos)))
                    status.values.append(KeyValue('Error', str(joint_state.error)))
                    status.values.append(KeyValue('Velocity', str(joint_state.velocity)))
                    status.values.append(KeyValue('Load', str(joint_state.load)))
                    status.values.append(KeyValue('Moving', str(joint_state.is_moving)))
                    status.values.append(KeyValue('Temperature', str(max_temp)))
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'
                    diag_msg.status.append(status)
                except:
                    pass
            #self.states_pub.publish(status)
            rate.sleep()

    def check_deps(self):
        controllers_still_waiting = []

        for i,(controller_name,deps,kls) in enumerate(self.waiting_meta_controllers):
            if not set(deps).issubset(self.controllers.keys()):
                controllers_still_waiting.append(self.waiting_meta_controllers[i])
                rospy.logwarn('[%s] not all dependencies started, still waiting for %s...' % (controller_name, str(list(set(deps).difference(self.controllers.keys())))))
            else:
                dependencies = [self.controllers[dep_name] for dep_name in deps]
                controller = kls(controller_name, dependencies)

                if controller.initialize():
                    controller.start()
                    self.controllers[controller_name] = controller
        self.waiting_meta_controllers = controllers_still_waiting[:]

    def start_controller(self, req):
        port_name = req.port_name
        package_path = req.package_path
        module_name = req.module_name
        class_name = req.class_name
        controller_name = req.controller_name
        self.start_controller_lock.acquire()
        if controller_name in self.controllers:
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Controller [%s] already started. If you want to restart it, call restart.' % controller_name)
        try:
            if module_name not in sys.modules:
                # import if module not previously imported
                package_module = __import__(package_path, globals(), locals(), [module_name], -1)
            else:
                # reload module if previously imported
                package_module = reload(sys.modules[package_path])
            controller_module = getattr(package_module, module_name)
        except ImportError, ie:
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Cannot find controller module. Unable to start controller %s\n%s' % (module_name, str(ie)))
        except SyntaxError, se:
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Syntax error in controller module. Unable to start controller %s\n%s' % (module_name, str(se)))
        except Exception, e:
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Unknown error has occured. Unable to start controller %s\n%s' % (module_name, str(e)))
        kls = getattr(controller_module, class_name)
        if port_name == 'meta':
            self.waiting_meta_controllers.append((controller_name,req.dependencies,kls))
            self.check_deps()
            self.start_controller_lock.release()
            return StartControllerResponse(True, '')
        if port_name != 'meta' and (port_name not in self.serial_proxies):
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Specified port [%s] not found, available ports are %s. Unable to start controller %s' % (port_name, str(self.serial_proxies.keys()), controller_name))
        controller = kls(self.serial_proxies[port_name].dxl_io, controller_name, port_name)
        if controller.initialize():
            controller.start()
            self.controllers[controller_name] = controller
            self.check_deps()
            self.start_controller_lock.release()
            return StartControllerResponse(True, 'Controller %s successfully started.' % controller_name)
        else:
            self.start_controller_lock.release()
            return StartControllerResponse(False, 'Initialization failed. Unable to start controller %s' % controller_name)

    def stop_controller(self, req):
        controller_name = req.controller_name
        self.stop_controller_lock.acquire()
        if controller_name in self.controllers:
            self.controllers[controller_name].stop()
            del self.controllers[controller_name]
            self.stop_controller_lock.release()
            return StopControllerResponse(True, 'controller %s successfully stopped.' % controller_name)
        else:
            self.self.stop_controller_lock.release()
            return StopControllerResponse(False, 'controller %s was not running.' % controller_name)

    def restart_controller(self, req):
        response1 = self.stop_controller(StopController(req.controller_name))
        response2 = self.start_controller(req)
        return RestartControllerResponse(response1.success and response2.success, '%s\n%s' % (response1.reason, response2.reason))

    def receive_joints_data(self, data):
	self.joint_state=data
	self.position_control()
	rospy.sleep(0.01)

    def position_control(self):	
	for i in range(len(self.joint_state.name)):
		if self.joint_state.name[i]=='joint_1':
			self.position = int(self.joint_state.position[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_position(1, self.position)
		if self.joint_state.name[i]=='joint_2':
			self.position = int(self.joint_state.position[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_position(2, self.position)
		if self.joint_state.name[i]=='joint_3':
			self.position = int(self.joint_state.position[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_position(3, self.position)
    def speed_control(self):
	for i in range(len(self.joint_state.name)):
		if self.joint_state.name[i]=='joint_1':
			self.speed = int(self.joint_state.velocity[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_speed(1, self.speed)
		if self.joint_state.name[i]=='joint_2':			
			self.speed = int(self.joint_state.velocity[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_speed(2, self.speed)
		if self.joint_state.name[i]=='joint_3':
			self.speed = int(self.joint_state.velocity[i])
			DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_position(3, self.speed)
    def torque_control(self):
	#self.joint_state.effort=[0, 0, 0]
	self.torque_1=int(self.joint_state.effort[0])
        self.torque_2=int(self.joint_state.effort[1])
        self.torque_3=int(self.joint_state.effort[2])  
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_goal_torque(1, self.torque_1)
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_goal_torque(2, self.torque_2)	
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_goal_torque(3, self.torque_3)		
    def torque_enable(self, req):
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(1, req)

	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(2, req)

	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(3, req)

        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
    '''def torque_disable(self):
	disabled=0;
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(1, False)
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(2, False)
	DynamixelIO('/dev/ttyACM0',57600,readback_echo=False).set_torque_enabled(3, False)'''

    def read_sensor(self):
    	self.status = CustomHand()
    	rate = rospy.Rate(5)
    	while not rospy.is_shutdown():
            try:
        	t = rospy.Time.now()
        	self.status.header.stamp =t
        	self.status.header.frame_id ='DENIRO/'
        	self.status.finger1=[0,0,0]
        	self.status.finger2=[0,0,0]
        	self.status.finger3=[0,0,0]
        	self.status.palm=[0,0]
        	self.status.finger1[0] = 1023-mcp_1.read_adc(5)
                self.status.finger1[1] = 1023-mcp_1.read_adc(4)
                self.status.finger1[2] = 1023-mcp_1.read_adc(3)
                self.status.finger2[0] = 1023-mcp_1.read_adc(2)
                self.status.finger2[1] = 1023-mcp_1.read_adc(1)
                self.status.finger2[2] = 1023-mcp_1.read_adc(0)
                self.status.finger3[0] = mcp_2.read_adc(7)
                self.status.finger3[1] = mcp_2.read_adc(6)
                self.status.finger3[2] = mcp_2.read_adc(5)
                self.status.palm[0] = mcp_2.read_adc(4)
                self.status.palm[1] = mcp_2.read_adc(3)
                rospy.loginfo(status)
            except:
                pass
            self.diagnostics_pub.publish(self.status)
            rate.sleep()
    def handActions():
	if action == Service.CLOSE_GRASP:
	   self.grab()
	elif action == Service.OPEN_GRASP:
           self.open_hand()
	elif action == Service.POINT_GRASP:
           self.point()
        elif action == Service.SQUEEZE_GRASP:
           self.squeeze()

    def predefined_position():
	DynamixelIO().set_position(self, 1, position)
        DynamixelIO().set_position(self, 2, position)
        DynamixelIO().set_position(self, 3, position)
    #pre-defined actions
    def grab():
	self.predefined_position()
    def open_hand():
	self.predefined_position()
    def point():
       self.predefined_position()
    def squeeze():
	self.predefined_position()

if __name__ == '__main__':
    try:
        manager = ControllerManager()
        rospy.spin()
    except rospy.ROSInterruptException: pass

