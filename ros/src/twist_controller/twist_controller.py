from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.kp_longidtudinal = rospy.get_param('~kp_longidtudinal', .3)
        self.ki_longidtudinal = rospy.get_param('~ki_longidtudinal', .1)
        self.kd_longidtudinal = rospy.get_param('~kd_longidtudinal', 0.)
        self.kp_lateral = rospy.get_param('~kp_lateral', .3)
        self.ki_lateral = rospy.get_param('~ki_lateral', .1)
        self.kd_lateral = rospy.get_param('~kd_lateral', 0.)
        
        self.pid_controller_longitudinal = PID(self.kp_longidtudinal, self.ki_longidtudinal, self.kd_longidtudinal, 0, 0.8)
##        self.pid_controller_lateral = PID(self.kp_lateral, self.ki_lateral, self.kd_lateral, -1, 1)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)
        self.lowpass_filter = LowPassFilter(tau=0.5, ts=0.02) # filtered_value = last_value * 25 / 26 + new_value / 26
        self.last_time = rospy.get_time()

    def control(self, current_linear_vel, dbw_enabled, cmd_linear_vel, cmd_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if dbw_enabled:
            current_linear_vel = self.lowpass_filter.filt(current_linear_vel)
            error_linear_vel = cmd_linear_vel - current_linear_vel
##            error_angular_vel = current_angular_vel - cmd_angular_vel
            current_time = rospy.get_time()
            sample_time = current_time - self.last_time
            self.last_time = current_time
            throttle = self.pid_controller_longitudinal.step(error_linear_vel, sample_time)
            brake = 0.
##            steer = self.pid_controller_lateral.step(error_angular_vel)
            steer = self.yaw_controller.get_steering(cmd_linear_vel, cmd_angular_vel, current_linear_vel)
            if cmd_linear_vel < .001 and current_linear_vel < .1: # hold on the car at standstill
                throttle = 0.
                brake = 400
            if throttle < .1 and error_linear_vel < 0: # we need to brake
                throttle = 0.
                decel = max(self.decel_limit, abs(error_linear_vel / sample_time))
                brake = abs(decel) * self.vehicle_mass * self.wheel_radius
                brake = 0. if brake < .1 else brake
        else:
            self.pid_controller_longitudinal.reset()
##            self.pid_controller_lateral.reset()
            throttle = brake = steer = 0.
        return throttle, brake, steer
