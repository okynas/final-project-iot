class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt):
        p_term = self.kp * error

        self.integral += error * dt
        i_term = self.ki * self.integral

        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative

        self.previous_error = error

        control = p_term + i_term + d_term

        return control

    def reset(self):
        self.previous_error = 0
        self.integral = 0