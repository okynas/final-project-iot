import cv2
import numpy as np
import math
from jetbot import Robot, Camera
from IPython.display import display, clear_output
import ipywidgets as widgets


class TrackFollower:
    def __init__(self):
        self.robot = Robot()
        self.camera = Camera.instance(width=224, height=224, fps=10)

        self.image_widget = widgets.Image(format='jpeg', description='Original')
        self.blur_widget = widgets.Image(format='jpeg', description='Blur')
        self.rgb_widget = widgets.Image(format='jpeg', description='RGB Mask')
        self.blur2_widget = widgets.Image(format='jpeg', description='Blur')
        self.edges_widget = widgets.Image(format='jpeg', description='Edges')
        self.roi_widget = widgets.Image(description='ROI')
        self.lines_detected_widget = widgets.Image(description="Lines detected")
        self.lines_widget = widgets.Image(description='Lines')
        self.heading_widget = widgets.Image(description='Heading')

        self.error_label = widgets.Label(value="Feil: 0")
        self.output_label = widgets.Label(value="PID Output: 0")

        self.kp_slider = widgets.FloatSlider(min=0.0, max=1, step=0.001, value=0.0, description='Kp')
        self.ki_slider = widgets.FloatSlider(min=0.0, max=1, step=0.001, value=0.000, description='Ki')
        self.kd_slider = widgets.FloatSlider(min=0.0, max=1, step=0.001, value=0.000, description='Kd')
        self.speed_slider = widgets.FloatSlider(min=0.0, max=1.0, step=0.01, value=0, description='Speed')

        layout = widgets.VBox([
            widgets.HBox([self.image_widget, self.blur_widget, self.rgb_widget]),
            widgets.HBox([self.blur2_widget, self.edges_widget, self.roi_widget]),
            widgets.HBox([self.lines_detected_widget, self.lines_widget, self.heading_widget]),
            widgets.HBox([
                widgets.VBox([self.kp_slider, self.ki_slider, self.kd_slider, self.speed_slider]),
                widgets.VBox([self.error_label, self.output_label])
            ])
        ])

        display(layout)

        self.history = []
        self.midline_history = []

        self.prev_error = 0
        self.integral = 0

        self.camera.observe(self.update_widget, names='value')
        print("Track Follower initialized and ready.")

    def convert_to_image_widget(self, frame):
        _, encoded_image = cv2.imencode('.jpg', frame)
        return encoded_image.tobytes()

    def detect_edges(self, frame):
        self.image_widget.value = self.convert_to_image_widget(frame)

        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        self.blur_widget.value = self.convert_to_image_widget(blur)

        lower_bound = np.array([30, 20, 40])
        upper_bound = np.array([120, 100, 120])

        mask = cv2.inRange(frame, lower_bound, upper_bound)
        self.rgb_widget.value = self.convert_to_image_widget(mask)

        blur = cv2.GaussianBlur(mask, (7, 7), 0)
        self.blur2_widget.value = self.convert_to_image_widget(blur)

        edges = cv2.Canny(blur, 50, 200)
        self.edges_widget.value = self.convert_to_image_widget(edges)

        return edges

    def region_of_interest(self, edges, region_height_fraction=0.5):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * (1 - region_height_fraction))),
            (0, int(height * (1 - region_height_fraction))),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        self.roi_widget.value = self.convert_to_image_widget(cropped_edges)
        return cropped_edges

    def select_representative_line(self, lines, side="left", center_x=None):
        """
        Velger den innerste linjen basert på helling og avstand til midten.
        """
        if len(lines) == 0:
            return None
        elif len(lines) == 1:
            return lines[0]
        else:
            def slope(line):
                x1, y1, x2, y2 = line
                return (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')  # Helling (unngå divisjon på null)

            def line_center(line):
                x1, _, x2, _ = line
                return (x1 + x2) / 2

            def is_closer_to_center(line):
                x1, _, x2, _ = line
                if side == "left":
                    return max(x1, x2) < center_x
                elif side == "right":
                    return min(x1, x2) > center_x

            inner_lines = [line for line in lines if is_closer_to_center(line)]

            if not inner_lines:
                inner_lines = lines

            representative_line = min(inner_lines, key=lambda line: abs(slope(line)))

            return representative_line

    def classify_lines(self, lines, image_center_x):
        """
        Klassifiserer linjesegmenter i venstre og høyre basert på deres midtpunkts x-koordinater.
        """
        left_lines = []
        right_lines = []

        for x1, y1, x2, y2 in lines:
            midpoint_x = (x1 + x2) / 2
            if midpoint_x < image_center_x:
                left_lines.append([x1, y1, x2, y2])
            else:
                right_lines.append([x1, y1, x2, y2])

        left_line = self.select_representative_line(left_lines, side="left", center_x=image_center_x)
        right_line = self.select_representative_line(right_lines, side="right", center_x=image_center_x)

        if left_line or right_line:
            self.history.append({'left': left_line, 'right': right_line})
            if len(self.history) > 10:
                self.history.pop(0)

        return [left_line] if left_line else [], [right_line] if right_line else []

    def calculate_dynamic_midline(self, left_lines, right_lines, image_height):
        """
        Dynamisk beregning av midtlinje basert på observerte venstre og høyre linjer.
        Tar hensyn til varierende bredder mellom linjene langs banen.
        """
        midline_points = []

        left_line = left_lines[0] if left_lines else None
        right_line = right_lines[0] if right_lines else None

        if left_line and right_line:
            left_mid_x = (left_line[0] + left_line[2]) / 2
            left_mid_y = (left_line[1] + left_line[3]) / 2
            right_mid_x = (right_line[0] + right_line[2]) / 2
            right_mid_y = (right_line[1] + right_line[3]) / 2

            mid_x = (left_mid_x + right_mid_x) / 2
            mid_y = (left_mid_y + right_mid_y) / 2
            midline_points.append((mid_x, mid_y))
        elif left_line:
            left_mid_x = (left_line[0] + left_line[2]) / 2
            left_mid_y = (left_line[1] + left_line[3]) / 2
            if self.midline_history:
                last_mid = self.midline_history[-1]
                avg_width = abs(last_mid[-1][0] - left_mid_x)  # Forrige bredde
                mid_x = left_mid_x + avg_width
                mid_y = left_mid_y
            else:
                mid_x = left_mid_x + 50
                mid_y = left_mid_y
            midline_points.append((mid_x, mid_y))
        elif right_line:
            right_mid_x = (right_line[0] + right_line[2]) / 2
            right_mid_y = (right_line[1] + right_line[3]) / 2
            if self.midline_history:
                last_mid = self.midline_history[-1]
                avg_width = abs(last_mid[-1][0] - right_mid_x)  # Forrige bredde
                mid_x = right_mid_x - avg_width
                mid_y = right_mid_y
            else:
                mid_x = right_mid_x - 50
                mid_y = right_mid_y
            midline_points.append((mid_x, mid_y))
        else:
            if self.midline_history:
                midline_points = self.midline_history[-1]
            else:
                midline_points = []

        if midline_points:
            self.midline_history.append(midline_points)
            if len(self.midline_history) > 10:
                self.midline_history.pop(0)

        return midline_points

    def detect_line_segments(self, cropped_edges):
        rho = 1
        theta = np.pi / 180
        min_threshold = 50
        line_segments = cv2.HoughLinesP(
            cropped_edges,
            rho,
            theta,
            min_threshold,
            np.array([]),
            minLineLength=30,
            maxLineGap=20
        )

        line_image = np.zeros_like(cropped_edges)

        if line_segments is not None:
            for segment in line_segments:
                for x1, y1, x2, y2 in segment:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        self.lines_detected_widget.value = self.convert_to_image_widget(line_image)

        return line_segments

    def process_image(self, frame):
        edges = self.detect_edges(frame)
        roi = self.region_of_interest(edges, region_height_fraction=0.5)
        line_segments = self.detect_line_segments(roi)


        if line_segments is not None:
            line_segments = [line[0] for line in line_segments]
            image_center_x = frame.shape[1] // 2
            left_lines, right_lines = self.classify_lines(line_segments, image_center_x)
            midline_points = self.calculate_dynamic_midline(left_lines, right_lines, frame.shape[0])

            combined_image = self.display_lines(frame, left_lines, right_lines, midline_points)
            return combined_image

        cv2.putText(frame, "No lines detected", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return frame

    def display_lines(self, frame, left_lines, right_lines, midline_points,
                      left_color=(255, 0, 0), right_color=(0, 255, 0), mid_color=(0, 0, 255), line_width=2):
        """
        Tegner venstre, høyre og midtlinje på bildet og kombinerer det med originalen.
        """
        line_image = np.zeros_like(frame)

        # Tegn venstre linjer
        if left_lines:
            for line in left_lines:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), left_color, line_width)

        # Tegn høyre linjer
        if right_lines:
            for line in right_lines:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), right_color, line_width)

        # Tegn midtlinjen
        if midline_points:
            if len(midline_points) > 1:
                for i in range(len(midline_points) - 1):
                    pt1 = (int(midline_points[i][0]), int(midline_points[i][1]))
                    pt2 = (int(midline_points[i + 1][0]), int(midline_points[i + 1][1]))
                    cv2.line(line_image, pt1, pt2, mid_color, line_width)
            else:
                pt = (int(midline_points[0][0]), int(midline_points[0][1]))
                cv2.circle(line_image, pt, 5, mid_color, -1)

        combined_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

        self.lines_widget.value = self.convert_to_image_widget(combined_image)

        return combined_image

    @staticmethod
    def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        steering_angle_radian = steering_angle * math.pi / 180.0

        x1 = int(width / 2)
        y1 = height

        x2 = int(x1 + height / 2 * math.tan(steering_angle_radian))
        y2 = int(height / 2)

        x2 = max(0, min(x2, width - 1))
        y2 = max(0, min(y2, height - 1))

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

        angle_text = f"Steering Angle: {steering_angle}°"
        cv2.putText(heading_image, angle_text, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        combined_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return combined_image

    def get_steering_angle(self, frame, middle_line):
        height, width, _ = frame.shape
        mid_x = int(width / 2)

        if middle_line:
            x1_middle, y1_middle, x2_middle, y2_middle = middle_line[0]

            delta_x_middle = x2_middle - x1_middle
            delta_y_middle = y2_middle - y1_middle
            middle_line_angle = math.atan2(delta_y_middle, delta_x_middle)

            ref_x1, ref_y1 = mid_x, height
            ref_x2, ref_y2 = mid_x, int(height / 2)
            delta_x_ref = ref_x2 - ref_x1
            delta_y_ref = ref_y2 - ref_y1
            ref_line_angle = math.atan2(delta_y_ref, delta_x_ref)

            angle_deviation = middle_line_angle - ref_line_angle
            angle_deviation_deg = int(angle_deviation * 180.0 / math.pi)

            return angle_deviation_deg
        else:
            return 0

    def get_offset_from_center(self, frame, middle_line):
        height, width, _ = frame.shape
        mid = int(width / 2)

        if middle_line:
            x1_middle, y1_middle, x2_middle, y2_middle = middle_line[0]
            line_center_x = (x1_middle + x2_middle) // 2

            offset = line_center_x - mid
            return offset
        else:
            return 0

    def follow_line(self, error):
        pid_output = self.pid_control(error)

        self.error_label.value = f"Feil: {error}"
        self.output_label.value = f"PID Output: {pid_output}"

        base_speed = self.speed_slider.value
        left_speed = max(min(base_speed + pid_output, 0.5), 0.0)
        right_speed = max(min(base_speed - pid_output, 0.5), 0.0)

        self.robot.set_motors(left_speed, right_speed)

    def pid_control(self, error):
        kp = self.kp_slider.value
        ki = self.ki_slider.value
        kd = self.kd_slider.value

        p = kp * error
        self.integral += error
        i = ki * self.integral
        d = kd * (error - self.prev_error)
        self.prev_error = error

        return p + i + d

    def update_widget(self, change):
        frame = change['new']
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        self.image_widget.value = self.convert_to_image_widget(frame)

        self.image_widget.value = self.convert_to_image_widget(frame)

        processed_frame = self.process_image(frame)
        if processed_frame is not None:
            self.heading_widget.value = self.convert_to_image_widget(processed_frame)
        else:
            print("Processed frame is None, skipping update.")

    @staticmethod
    def bgr8_to_jpeg(value):
        return bytes(cv2.imencode('.jpg', value)[1])

    def stop(self):
        self.robot.stop()
        self.camera.unobserve_all()
        self.camera.close()
        print("Track Follower stopped and resources released.")
