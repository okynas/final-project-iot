import threading
import time
from jetbot import Robot, Camera
from Manager.RoadFollower import RoadFollower, RobotState
from Functions.USBCamera import USBCamera, CameraWidget
import ipywidgets as widgets
from IPython.display import display
import cv2
import traceback

class RobotManager:
    def __init__(self, robot, camera, base_speed=0.3, min_speed=0.1, pid_params=None):
        self.robot = robot
        self.camera = camera
        self.base_speed = base_speed
        self.min_speed = min_speed

        self.pid_params = pid_params or {"kp": 0.016, "ki": 0.000, "kd": 0.004}
        self.road_follower = RoadFollower(
            robot=self.robot,
            camera=self.camera,
            base_speed=self.base_speed,
            min_speed=self.min_speed,
            pid_params=pid_params
        )

        self.camera_feed_thread = None

        # Widgets
        self.start_button = widgets.Button(description='Start', button_style='success')
        self.stop_button = widgets.Button(description='Stop', button_style='danger')
        self.status_label = widgets.Label(value="Status: Idle")
        self.mode_label = widgets.Label(value="Mode: N/A")
        self.angle_label = widgets.Label(value="Angle: N/A")
        self.steering_label = widgets.Label(value="Steering Angle: N/A")
        self.hough_label = widgets.Label(value="Hough Transform: Unknown")
        self.image_widget = widgets.Image(format='png', width=400, height=400)
        self.masked_image_widget = widgets.Image(format='png', width=224, height=224)
        self.masked_cleand_image_widget = widgets.Image(format='png', width=224, height=224)
        self.log_area = widgets.Textarea(value="", description="Log:", layout=widgets.Layout(width='100%', height='200px'))

        self.max_speed_slider = widgets.FloatSlider(value=self.base_speed, min=0.1, max=1.0, step=0.05, description='Max Speed', readout_format='.2f')
        self.min_speed_slider = widgets.FloatSlider(value=self.min_speed, min=0.1, max=0.5, step=0.05, description='Min Speed', readout_format='.2f')
        self.kp_slider = widgets.FloatSlider(value=self.pid_params['kp'], min=0., max=0.1, step=0.001, description='Kp', readout_format='.3f')
        self.ki_slider = widgets.FloatSlider(value=self.pid_params['ki'], min=0, max=0.01, step=0.0001, description='Ki', readout_format='.3f')
        self.kd_slider = widgets.FloatSlider(value=self.pid_params['kd'], min=0, max=0.1, step=0.001, description='Kd', readout_format='.3f')

        self.max_speed_slider.observe(self.update_values, names='value')
        self.min_speed_slider.observe(self.update_values, names='value')
        self.kp_slider.observe(self.update_values, names='value')
        self.ki_slider.observe(self.update_values, names='value')
        self.kd_slider.observe(self.update_values, names='value')

        self.start_button.on_click(self.on_start_click)
        self.stop_button.on_click(self.on_stop_click)

        control_widgets = widgets.VBox([
            widgets.HBox([self.start_button, self.stop_button,]),
            self.max_speed_slider,
            self.min_speed_slider,
            self.kp_slider,
            self.ki_slider,
            self.kd_slider,
            self.status_label
        ])

        status_widgets = widgets.VBox([
            self.mode_label,
            self.steering_label,
            self.angle_label,
            self.hough_label
        ])

        debug_image_widget = widgets.VBox([
            widgets.HBox([
                self.masked_image_widget,
                self.masked_cleand_image_widget,
            ])
        ])

        display(widgets.VBox([
            widgets.HBox([
                self.image_widget,
                widgets.VBox([control_widgets, status_widgets])
            ]),
            debug_image_widget
        ]))

        # Start kameraoppdateringen ved init
        self.start_camera_feed()

    def log_message(self, message):
        """Logger meldinger til tekstområdet."""
        self.log_area.value = f"{self.log_area.value}\n{message}".strip()

    def convert_image_to_bytes(self, image):
        """Konverterer et bilde til bytes for visning i en widget."""
        _, buffer = cv2.imencode('.png', image)
        return buffer.tobytes()

    def start_road_following(self):
        """Starter linjefølging."""
        if self.road_follower.state != RobotState.RUNNING:
            self.road_follower.change_state(RobotState.RUNNING)
            threading.Thread(target=self.road_follower.follow_line_loop, daemon=True).start()
            self.status_label.value = "Status: Line following started."
            self.log_message("Line following started.")

    def stop_road_following(self):
        """Stopper linjefølging."""
        if self.road_follower.state == RobotState.RUNNING:
            self.road_follower.change_state(RobotState.IDLE)
            self.status_label.value = "Status: Line following stopped."
            self.log_message("Line following stopped.")

    def start_camera_feed(self):
        """Starter kamerafeeden i en egen tråd."""
        if self.camera_feed_thread is None or not self.camera_feed_thread.is_alive():
            self.camera_feed_thread = threading.Thread(target=self.update_camera_feed, daemon=True)
            self.camera_feed_thread.start()
            self.status_label.value = "Status: Camera feed running."
            self.log_message("Camera feed running.")

    def update_camera_feed(self):
        """Oppdaterer kamerafeeden."""
        while True:
            try:
                result = self.road_follower.detector.process_frame()
                self.status_label.value = f"Status: {result['status_message']}"
                self.mode_label.value = f"Mode: {result['mode']}"
                self.angle_label.value = f"Angle: {result['angle']:.2f}°"
                self.steering_label.value = f"Steering Angle: {result['steering_angle']:.2f}"
                self.hough_label.value = f"Hough Transform: {result.get('hough_transform', 'Unknown')}"
                self.image_widget.value = self.convert_image_to_bytes(result['processed_images']['result'])
                self.masked_image_widget.value = self.convert_image_to_bytes(result['processed_images']['masked'])
                self.masked_cleand_image_widget.value = self.convert_image_to_bytes(result['processed_images']['masked_cleand'])

                time.sleep(0.1)
            except Exception as e:
                self.log_message(f"Error in camera feed: {e}")
                self.status_label.value = "Status: Camera feed error"
                break

    def on_start_click(self, _):
        """Klikk på Start-knappen."""
        self.start_road_following()

    def on_stop_click(self, _):
        """Klikk på Stop-knappen."""
        self.stop_road_following()
        self.log_message("Stop button clicked.")

    def update_values(self, _):
        """Oppdaterer RoadFollower-parametere basert på sliderverdier."""
        # Oppdater hastigheter
        self.road_follower.base_speed = self.max_speed_slider.value
        self.road_follower.min_speed = self.min_speed_slider.value

        # Oppdater PID-parametere
        self.road_follower.pid.kp = self.kp_slider.value
        self.road_follower.pid.ki = self.ki_slider.value
        self.road_follower.pid.kd = self.kd_slider.value

        # Logg oppdateringene
        self.log_message(
            f"Updated: Max Speed={self.max_speed_slider.value}, Min Speed={self.min_speed_slider.value}, "
            f"Kp={self.kp_slider.value}, Ki={self.ki_slider.value}, Kd={self.kd_slider.value}"
        )

if __name__ == "__main__":
    try:
        robot = Robot()
        camera = USBCamera.instance()
        manager = RobotManager(robot, camera, base_speed=0.3, min_speed=0.20)
    except Exception as e:
        print(f"Error initializing RobotManager: {e}")
        traceback.print_exc()
