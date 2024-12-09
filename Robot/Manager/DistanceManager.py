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
        self.log_callback = log_callback if log_callback else print
        self.target_distance = target_distance

        self.platoon_speed = road_follower.base_speed
        self.enabled = True
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
        while True:
            if self.enabled:
                distance = self.distance_sensor.get_distance()

                if distance is not None and self.min_distance <= distance <= self.max_distance:
                    if distance > self.target_distance + 50:
                        new_speed = min(self.max_speed, self.road_follower.base_speed + 0.05)
                        self.log(f"Avstand {distance} mm er for stor. Øker hastigheten til {new_speed:.2f}.")
                    elif distance < self.target_distance - 50:
                        new_speed = max(self.min_speed, self.road_follower.base_speed - 0.05)
                        self.log(f"Avstand {distance} mm er for liten. Reduserer hastigheten til {new_speed:.2f}.")
                    else:
                        new_speed = self.platoon_speed
                        self.log(f"Avstand {distance} mm er innenfor målområdet. Beholder hastigheten {new_speed:.2f}.")

                    self.road_follower.base_speed = new_speed
                    self.recent_speeds.append(new_speed)
                else:
                    if self.recent_speeds:
                        average_speed = sum(self.recent_speeds) / len(self.recent_speeds)
                        self.road_follower.base_speed = average_speed
                        self.log(f"Ugyldig avstand. Bruker gjennomsnittlig hastighet: {average_speed:.2f}.")
                    else:
                        self.road_follower.base_speed = self.platoon_speed
                        self.log(
                            f"Ugyldig avstand og ingen historikk. Beholder platoon_speed: {self.platoon_speed:.2f}.")
            else:
                self.road_follower.base_speed = self.platoon_speed
                self.log("Avstandsmåling deaktivert eller leder. Base speed satt til standard.")

            time.sleep(0.1)
