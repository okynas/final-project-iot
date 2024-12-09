import time
import numpy as np
from enum import Enum, auto
from Functions.LineDetector import LineDetector
from Functions.PIDController import PIDController

class RobotState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ERROR = auto()

class RoadFollower:
    def __init__(self, robot, camera, base_speed=0.4, min_speed=0.1, pid_params=None, log_callback=None):
        self.robot = robot
        self.camera = camera
        self.detector = LineDetector(camera=self.camera)
        self.state = RobotState.IDLE

        pid_params = pid_params or {"kp": 0.016, "ki": 0.0000, "kd": 0.004}
        self.pid = PIDController(kp=pid_params["kp"], ki=pid_params["ki"], kd=pid_params["kd"])

        self._steering_angle = 0
        self.base_speed = base_speed
        self.min_speed = min_speed
        self.is_leader = True
        self.max_steering = 1
        self.previous_time = time.time()
        self.log_callback = log_callback if log_callback else print

    @property
    def steering_angle(self):
        """Gir tilgang til styringsvinkelen."""
        return self._steering_angle

    def set_is_leader(self, is_leader):
        self.is_leader = is_leader

    def set_steering_angle(self, steering_angle):
        self._steering_angle = steering_angle

    def log(self, message):
        """Logger en melding via log_callback."""
        self.log_callback(message)

    def change_state(self, new_state):
        """Oppdaterer robotens tilstand."""
        if new_state != self.state:
            self.log(f"Endrer tilstand fra {self.state.name} til {new_state.name}")
            self.state = new_state

    def steer(self, steering_value, speed):
        if self.state != RobotState.RUNNING:
            return

        steering_value = np.clip(steering_value, -1, 1)
        left_speed = speed * (1 + min(steering_value, 0))
        right_speed = speed * (1 - max(steering_value, 0))
        try:
            self.robot.left_motor.value = float(left_speed)
            self.robot.right_motor.value = float(right_speed)
        except AttributeError as e:
            self.log(f"Feil ved styring: {e}")

    def follow_line_single(self, speed, min_speed):
        if self.state != RobotState.RUNNING:
            return

        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        try:
            result = self.detector.process_frame()
            self._steering_angle = result['steering_angle']

            control_value = self.pid.compute(self._steering_angle, dt)
            control_value = np.clip(control_value, -self.max_steering, self.max_steering)

            scaling_factor = np.exp(-abs(control_value) * 1)
            adjusted_speed = max(speed * scaling_factor, min_speed)

            self.steer(control_value, self.base_speed)
        except Exception as e:
            self.log(f"Feil i follow_line_single: {e}")
            self.change_state(RobotState.ERROR)

    def follow_line_loop(self):
        """Hovedløkken for linjefølging med tilstandshåndtering."""
        try:
            while True:
                if self.state == RobotState.RUNNING:
                    if self.is_leader:
                        self.follow_line_single(speed=self.base_speed, min_speed=self.min_speed)
                    else:
                        self.follow_robot(speed=self.base_speed, min_speed=self.min_speed)
                elif self.state in {RobotState.PAUSED, RobotState.ERROR, RobotState.IDLE}:
                    self.stop()
                time.sleep(0.1)
        except Exception as e:
            self.log(f"Uventet feil i follow_line_loop: {e}")
            self.change_state(RobotState.ERROR)

    def follow_robot(self, speed, min_speed):
        """Følg roboten foran (følger-oppførsel)."""
        if self.state != RobotState.RUNNING:
            return

        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        try:
            control_value = self.pid.compute(self._steering_angle, dt)
            control_value = np.clip(control_value, -self.max_steering, self.max_steering)

            scaling_factor = np.exp(-abs(control_value) * 1)
            adjusted_speed = max(speed * scaling_factor, min_speed)

            self.steer(control_value, adjusted_speed)
        except Exception as e:
            self.log(f"Feil i follow_line_single: {e}")
            self.change_state(RobotState.ERROR)

    def stop(self):
        """Stopper robotens motorer."""
        try:
            self.robot.left_motor.value = 0.0
            self.robot.right_motor.value = 0.0
            self.log("Robot stoppet.")
        except AttributeError as e:
            self.log(f"Kunne ikke stoppe motorene: {e}")

