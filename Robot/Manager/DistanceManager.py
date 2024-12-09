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
        self.max_speed = 0.25
        self.distance = 0

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
                self.distance = self.distance_sensor.get_distance()

                if self.distance is not None and self.min_distance <= self.distance <= self.max_distance:
                    if self.distance > self.target_distance + 100:
                        # Øk hastigheten
                        new_speed = self.road_follower.base_speed + 0.02
                        self.log(
                            f"Avstand {self.distance} mm er for stor. Beregnet hastighet før begrensning: {new_speed:.2f}.")
                    elif self.distance < self.target_distance - 100:
                        # Reduser hastigheten
                        new_speed = self.road_follower.base_speed - 0.02
                        self.log(
                            f"Avstand {self.distance} mm er for liten. Beregnet hastighet før begrensning: {new_speed:.2f}.")
                    else:
                        # Hold nåværende platoon_speed
                        new_speed = self.platoon_speed
                        self.log(
                            f"Avstand {self.distance} mm er innenfor målområdet. Beholder hastigheten {new_speed:.2f}.")

                    # Begrens hastigheten til tillatt område
                    new_speed = max(self.min_speed, min(self.max_speed, new_speed))
                    self.log(f"Hastigheten etter begrensning: {new_speed:.2f}.")

                    # Oppdater base_speed og historikk
                    self.road_follower.base_speed = new_speed
                    self.recent_speeds.append(new_speed)
                else:
                    # Håndter ugyldig avstand
                    if self.recent_speeds:
                        average_speed = sum(self.recent_speeds) / len(self.recent_speeds)
                        self.road_follower.base_speed = average_speed
                        self.log(f"Ugyldig avstand. Bruker gjennomsnittlig hastighet: {average_speed:.2f}.")
                    else:
                        self.road_follower.base_speed = self.platoon_speed
                        self.log(
                            f"Ugyldig avstand og ingen historikk. Beholder platoon_speed: {self.platoon_speed:.2f}.")
            else:
                # Avstandsmåling deaktivert
                self.road_follower.base_speed = self.platoon_speed
                self.log("Avstandsmåling deaktivert eller leder. Base speed satt til standard.")

            time.sleep(0.1)
