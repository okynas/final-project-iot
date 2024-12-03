import time
import collections
from Functions.DistanceSensor import DistanceSensor
from Functions.PIDController import PIDController

class DistanceManager:
    def __init__(self, i2c_bus, i2c_address, road_follower, log_callback=None, target_distance=300, pid_params=None):
        """
        Håndterer avstandsbasert logikk for platooning.
        """
        self.distance_sensor = DistanceSensor(i2c_bus=i2c_bus, i2c_address=i2c_address, ranging_mode=1)
        self.road_follower = road_follower
        self.log_callback = log_callback or (lambda msg: None)
        self.target_distance = target_distance

        self.platoon_speed = road_follower.base_speed
        self.enabled = False
        self.min_distance = 0
        self.max_distance = 1000
        self.min_speed = 0.1
        self.max_speed = 0.6

        self.recent_speeds = collections.deque(maxlen=10)

        pid_params = pid_params or {"kp": 0.01, "ki": 0.005, "kd": 0.001}
        self.pid_controller = PIDController(
            kp=pid_params["kp"],
            ki=pid_params["ki"],
            kd=pid_params["kd"]
        )

    def log(self, message):
        """Logger melding hvis log_callback er satt."""
        if self.log_callback:
            self.log_callback(message)

    def enable_distance_measurement(self, enable):
        """Aktiverer eller deaktiverer avstandsmåling."""
        self.enabled = enable

    def update_platoon_speed(self, speed):
        """Oppdaterer hastigheten til bilen foran."""
        self.platoon_speed = speed
        self.log(f"Oppdatert hastighet fra bilen foran: {speed:.2f}")

    def monitor_distance(self):
        """Overvåker avstanden og justerer base_speed for å oppnå ønsket avstand."""
        previous_time = time.time()

        while True:
            if self.enabled:
                distance = self.distance_sensor.get_distance()
                current_time = time.time()
                dt = current_time - previous_time
                previous_time = current_time

                if distance is not None and self.min_distance <= distance <= self.max_distance:
                    error = distance - self.target_distance

                    speed_adjustment = self.pid_controller.compute(error, dt)

                    adjusted_speed = min(self.max_speed, max(self.min_speed, self.platoon_speed + speed_adjustment))

                    self.road_follower.base_speed = adjusted_speed
                    self.log(
                        f"Avstand: {distance} mm | Feil: {error} | PID-justering: {speed_adjustment:.2f} | "
                        f"Ny base_speed: {adjusted_speed:.2f}")
                else:
                    if self.recent_speeds:
                        average_speed = sum(self.recent_speeds) / len(self.recent_speeds)
                        self.road_follower.base_speed = average_speed
                        self.log(
                            f"Ingen gyldig avstandsmåling. Bruker gjennomsnittlig hastighet: {average_speed:.2f}")
                    else:
                        self.road_follower.base_speed = self.platoon_speed
                        self.log(
                            f"Ingen gyldig avstandsmåling eller historikk. Beholder platoon_speed: {self.platoon_speed:.2f}")
            else:
                self.road_follower.base_speed = self.platoon_speed or 0.3
                self.log("Avstandsmåling deaktivert eller leder. Base speed satt til standard.")

            time.sleep(0.1)

