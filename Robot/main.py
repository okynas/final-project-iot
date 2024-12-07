import threading
from collections import deque
from jetbot import Robot, Camera
from Manager.RoadFollower import RoadFollower, RobotState
from Manager.PlatoonManager import PlatoonManager
from Manager.QRCodeManager import QRCodeManager
from Manager.DistanceManager import DistanceManager
import ipywidgets as widgets
from IPython.display import display

class RobotManager:
    def __init__(self, robot, camera, mqtt_broker_ip, port, robot_id, base_speed, min_speed):
        self.robot = robot
        self.camera = camera
        self.base_speed = base_speed
        self.min_speed = min_speed
        self.last_log_message = {}

        self.log_areas = {
            "road_follower": widgets.Textarea(
                value="", placeholder="RoadFollower logs will appear here.",
                layout=widgets.Layout(height="150px", width="400px")
            ),
            "platoon_manager": widgets.Textarea(
                value="", placeholder="PlatoonManager logs will appear here.",
                layout=widgets.Layout(height="150px", width="400px")
            ),
            "qr_manager": widgets.Textarea(
                value="", placeholder="QRCodeManager logs will appear here.",
                layout=widgets.Layout(height="150px", width="400px")
            ),
            "distance_manager": widgets.Textarea(
                value="", placeholder="DistanceManager logs will appear here.",
                layout=widgets.Layout(height="150px", width="400px")
            ),
        }

        self.road_follower = RoadFollower(
            robot=self.robot,
            camera=self.camera,
            base_speed=self.base_speed,
            min_speed=self.min_speed,
            pid_params={"kp": 0.016, "ki": 0.000, "kd": 0.004},
            log_callback = lambda msg: self.log_message(msg, "road_follower")
        )

        self.distance_manager = DistanceManager(
            i2c_bus=1,
            i2c_address=0x29,
            road_follower=self.road_follower,
            log_callback=lambda msg: self.log_message(msg, "distance_manager"),
            target_distance=300,
            pid_params={"kp": 0.01, "ki": 0.00, "kd": 0.001}
        )

        self.platoon_manager = PlatoonManager(
            robot_id=robot_id,
            mqtt_broker=mqtt_broker_ip,
            port=port,
            road_follower=self.road_follower,
            distance_manager=self.distance_manager,
            log_callback=lambda msg: self.log_message(msg, "platoon_manager")
        )

        self.qr_manager = QRCodeManager(
            camera=self.camera,
            platoon_manager=self.platoon_manager,
            distance_manager=self.distance_manager,
            log_callback=lambda msg: self.log_message(msg, "qr_manager"),
            timeout=2.0
        )

        self.qr_thread = threading.Thread(target=self.qr_manager.process_qr_codes, daemon=True)
        self.platoon_thread = threading.Thread(target=self.platoon_manager.start, daemon=True)
        self.roadfollowing_thread = threading.Thread(target=self.road_follower.follow_line_loop, daemon=True)
        self.distance_thread = threading.Thread(target=self.distance_manager.monitor_distance, daemon=True)

        self.max_speed_slider = widgets.FloatSlider(value=self.base_speed, min=0.1, max=0.6, step=0.01,
                                                    description='Max Speed:')
        self.min_speed_slider = widgets.FloatSlider(value=self.min_speed, min=0.05, max=0.3, step=0.01,
                                                    description='Min Speed:')

        self.start_button = widgets.Button(description="Start")
        self.stop_button = widgets.Button(description="Stop")

        self.setup_ui()

        self.qr_thread.start()
        self.platoon_thread.start()
        self.roadfollowing_thread.start()
        self.distance_thread.start()

    def setup_ui(self):
        """Setter opp UI-komponentene og kobler hendelser."""
        self.start_button.on_click(self.start_road_following)
        self.stop_button.on_click(self.stop_road_following)
        self.max_speed_slider.observe(self.update_values, names='value')
        self.min_speed_slider.observe(self.update_values, names='value')

        display(widgets.VBox([
            widgets.VBox([
                widgets.HBox([
                    self.start_button,
                    self.stop_button,
                ]),
                self.max_speed_slider,
                self.min_speed_slider,
            ]),
            widgets.VBox([
                widgets.HBox([
                    widgets.VBox([widgets.Label("RoadFollower"), self.log_areas["road_follower"]]),
                    widgets.VBox([widgets.Label("PlatoonManager"), self.log_areas["platoon_manager"]])
                ]),
                widgets.HBox([
                    widgets.VBox([widgets.Label("QRCodeManager"), self.log_areas["qr_manager"]]),
                    widgets.VBox([widgets.Label("DistanceManager"), self.log_areas["distance_manager"]])
                ])
            ])
        ]))

    def log_message(self, message, manager_name):
        """Logger meldinger i riktig tekstområde."""
        max_lines = 20
        if message != self.last_log_message.get(manager_name):
            log_area = self.log_areas[manager_name]
            log_area.value = f"{log_area.value}\n{message}".strip()
            self.last_log_message[manager_name] = message
            lines = log_area.value.split("\n")
            if len(lines) > max_lines:
                log_area.value = "\n".join(lines[-max_lines:])

    def start_road_following(self, _=None):
        """Starter linjefølging."""
        if self.road_follower.state != RobotState.RUNNING:
            self.road_follower.change_state(RobotState.RUNNING)
            self.log_message("Linjen følger startet.")

    def stop_road_following(self, _=None):
        """Stopper linjefølging."""
        if self.road_follower.state == RobotState.RUNNING:
            self.road_follower.change_state(RobotState.IDLE)
            self.log_message("Linjen følger stoppet.")

    def update_values(self, _):
        """Oppdaterer RoadFollower-parametere basert på sliderverdier."""
        self.road_follower.base_speed = self.max_speed_slider.value
        self.road_follower.min_speed = self.min_speed_slider.value

        self.log_message(
            f"Updated: Max Speed={self.max_speed_slider.value}, Min Speed={self.min_speed_slider.value}, "
        )