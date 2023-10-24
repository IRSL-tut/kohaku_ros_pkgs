# -*- coding: utf-8 -*-
import rospy
import json
from geometry_msgs.msg import Vector3, Wrench
from hr4c_driver import KohakuModel4
from hr4c_msgs.srv import (
    GetInt8Array,
    GetInt8ArrayResponse,
    SetFloat64,
    SetFloat64Response,
    SetInt8Array,
    SetInt8ArrayResponse,
    SetInt8,
    SetInt8Response,
    SetJointTrajectory,
    SetJointTrajectoryResponse,
    SetPose,
    SetPoseResponse,
    GetPose,
    GetPoseResponse,
    SetForceMoment,
    SetForceMomentResponse,
    GetForceMoment,
    GetForceMomentResponse,
    GetMotorStatus,
    GetMotorStatusResponse,
    GetInterpolationStatus,
    GetInterpolationStatusResponse
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse


class KohakuROSDriverNode(object):
    def __init__(self):
        rospy.init_node('kohaku_ros_driver_node')

        # parameters
        ipaddr = rospy.get_param('~ip_addr', '192.168.1.100')
        lr = rospy.get_param('~lr', 'r')
        version = rospy.get_param('~version', 'v1')
        finger = rospy.get_param('~finger', 'normal')
        self._joint_names = rospy.get_param('~joint_names', ['j1',
                                                             'j2',
                                                             'j3',
                                                             'j4',
                                                             'j5',
                                                             'j6',
                                                             'j7',
                                                             'j8'
                                                             ])
        self._sampling_rate = rospy.get_param('~sampling_rate', 100)

        # attributes
        self._kohaku_driver = KohakuModel4(ipaddr,
                                           lr=lr,
                                           version=version,
                                           finger=finger,
                                           sleep_func=rospy.sleep)
        self._joint_angles = None
        self._joint_currents = None
        self._joint_torques = None
        self._joint_speeds = None

        # publishers
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=100)
        self._current_pub = rospy.Publisher('joint_currents', Float64MultiArray, queue_size=100)

        # services
        rospy.Service('servo_all_on', Empty, self.handle_servo_all_on)
        rospy.Service('servo_all_off', Empty, self.handle_servo_all_off)
        rospy.Service('check_interpolation', GetInterpolationStatus, self.handle_check_interpolation)
        rospy.Service('go_to_home_position', Empty, self.handle_go_homepos)
        rospy.Service('go_to_rest_position', Empty, self.handle_go_restpos)
        rospy.Service('alarm_reset', SetInt8, self.handle_alarm_reset)
        rospy.Service('servo_on', SetInt8, self.handle_servo_on)
        rospy.Service('servo_off', SetInt8, self.handle_servo_off)
        rospy.Service('set_control_mode', SetInt8Array, self.handle_set_control_mode)
        rospy.Service('get_control_mode', GetInt8Array, self.handle_get_control_mode)
        rospy.Service('set_joint_trajectory', SetJointTrajectory, self.handle_set_joint_trajectory)
        rospy.Service('set_pose', SetPose, self.handle_set_pose)
        rospy.Service('get_pose', GetPose, self.handle_get_pose)
        rospy.Service('set_force_moment', SetForceMoment, self.handle_set_force_moment)
        rospy.Service('get_force_moment', GetForceMoment, self.handle_get_force_moment)
        rospy.Service('force_stop', SetInt8, self.handle_force_stop)
        rospy.Service('get_motor_status', GetMotorStatus, self.handle_get_motor_status)
        rospy.Service('enable_zerog_mode', SetBool, self.handle_enable_zerog_mode)
        rospy.Service('grasp', Empty, self.handle_grasp)
        rospy.Service('open_hand', SetFloat64, self.handle_open_hand)

        # initialize device
        self._kohaku_driver.open_device()

    def shutdown(self):
        self._kohaku_driver.close_device()

    def handle_servo_all_on(self, req):
        self._kohaku_driver.servo_all_on()
        return EmptyResponse()

    def handle_servo_all_off(self, req):
        self._kohaku_driver.servo_all_off()
        return EmptyResponse()

    def handle_check_interpolation(self, req):
        ret =self._kohaku_driver.check_interpolation()
        return GetInterpolationStatusResponse(ret)

    def handle_go_homepos(self, req):
        go_home_time = 3.0
        self._kohaku_driver.go_to_home_position(goal_time=go_home_time,
                                                wait_interpolation=False)
        rospy.sleep(go_home_time)

        return EmptyResponse()

    def handle_go_restpos(self, req):
        go_rest_time = 3.0
        self._kohaku_driver.go_to_rest_position(goal_time=go_rest_time,
                                                wait_interpolation=False)
        rospy.sleep(go_rest_time)
        self._kohaku_driver.servo_all_off()

        return EmptyResponse()

    def handle_alarm_reset(self, req):
        self._kohaku_driver.alarm_reset(req.arg)
        return SetInt8Response(result=True)

    def handle_servo_on(self, req):
        self._kohaku_driver.servo_on(req.arg)
        return SetInt8Response(result=True)

    def handle_servo_off(self, req):
        self._kohaku_driver.servo_off(req.arg)
        return SetInt8Response(result=True)

    def handle_set_control_mode(self, req):
        control_modes = req.arg
        if len(control_modes) != self._kohaku_driver.get_dof():
            rospy.logwarn('Invalid data number: ' + str(len(control_modes)))
            return SetInt8ArrayResponse(result=False)
        else:
            self._kohaku_driver.set_control_mode(control_modes)

        return SetInt8ArrayResponse(result=True)

    def handle_get_control_mode(self, req):
        control_modes = self._kohaku_driver.get_control_mode()

        return GetInt8ArrayResponse(res=control_modes)

    def handle_set_joint_trajectory(self, req):
        self._kohaku_driver.set_joint_trajectory(req.target_references,
                                                 req.goal_time,
                                                 mask=req.mask,
                                                 interpolation_method=req.interpolation_method,
                                                 relative=req.relative,
                                                 wait_interpolation=False)
        if req.wait_interpolation:
            rospy.sleep(req.goal_time)

        return SetJointTrajectoryResponse()

    def handle_set_pose(self, req):
        self._kohaku_driver.set_pose(req.position.x,
                                     req.position.y,
                                     req.position.z,
                                     req.goal_time,
                                     interpolation_method=req.interpolation_method,
                                     linear_motion=req.linear_motion,
                                     wait_interpolation=False)
        if req.wait_interpolation:
            rospy.sleep(req.goal_time)

        return SetPoseResponse()

    def handle_get_pose(self, req):
        pos = self._kohaku_driver.get_pose()
        return GetPoseResponse(Vector3(x=pos[0], y=pos[1], z=pos[2]))

    def handle_set_force_moment(self, req):
        self._kohaku_driver.set_force_moment(req.force_moment.force.x,
                                             req.force_moment.force.y,
                                             req.force_moment.force.z,
                                             req.force_moment.torque.x,
                                             req.force_moment.torque.y,
                                             req.force_moment.torque.z)
        return SetForceMomentResponse()

    def handle_get_force_moment(self, req):
        torques = self._kohaku_driver.get_arm_joint_torque()
        fm_list = self._kohaku_driver.get_force_moment(torques)
        return GetForceMomentResponse(force_moment=Wrench(force=Vector3(x=fm_list[0],
                                                                        y=fm_list[1],
                                                                        z=fm_list[2]),
                                                          torque=Vector3(x=fm_list[3],
                                                                         y=fm_list[4],
                                                                         z=fm_list[5])))

    def handle_force_stop(self, req):
        self._kohaku_driver.force_stop(method=req.arg)
        return SetInt8Response()

    def handle_get_motor_status(self, req):
        motor_status_dict = self._kohaku_driver.get_motor_status()
        motor_status_str = json.dumps(motor_status_dict)
        return GetMotorStatusResponse(motor_status=motor_status_str)

    def handle_enable_zerog_mode(self, req):
        self._kohaku_driver.enable_zerog_mode(req.data)
        return SetBoolResponse(success=True, message='')

    def handle_grasp(self, req):
        self._kohaku_driver.close_hand_until_contact()
        return EmptyResponse()

    def handle_open_hand(self, req):
        goal_time = 2.0
        self._kohaku_driver.open_hand([req.arg, req.arg],
                                      goal_time,
                                      wait_interpolation=False)
        rospy.sleep(goal_time)
        return SetFloat64Response(result=True)

    def publish_joint_state(self, angles, velocities, efforts):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self._joint_names
        js.position = angles
        js.velocity = velocities
        js.effort = efforts
        self._joint_pub.publish(js)

    def publish_joint_current(self, currents):
        self._current_pub.publish(Float64MultiArray(data=currents))

    def update_sensor_data(self):
        self._joint_angles = self._kohaku_driver.get_joint_angle()
        self._joint_currents = self._kohaku_driver.get_joint_current()
        self._joint_torques = self._kohaku_driver.get_joint_torque()
        self._joint_speeds = self._kohaku_driver.get_joint_speed()

    def run(self):
        r = rospy.Rate(self._sampling_rate)
        while not rospy.is_shutdown():
            self.update_sensor_data()
            self.publish_joint_state(self._joint_angles,
                                     self._joint_speeds,
                                     self._joint_torques)
            self.publish_joint_current(self._joint_currents)
            r.sleep()
