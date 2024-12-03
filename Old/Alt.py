import cv2
import numpy as np
import math
from jetbot import Robot, Camera
from IPython.display import display
from collections import Counter
import ipywidgets as widgets
import time


class TrackFollower:
    def __init__(self):
        self.robot = Robot()
        self.camera = Camera.instance(width=112, height=112, fps=10)
        self.lower_bound = np.array([30, 20, 40])
        self.upper_bound = np.array([120, 100, 120])

        self.lines_widget = widgets.Image(description='Lines')
        self.heading_widget = widgets.Image(description='Heading')
        self.error_label = widgets.Label(value="Feil: 0")
        self.output_label = widgets.Label(value="PID Output: 0")
        self.angle_label = widgets.Label(value="Angle Output: 0")
        self.offset_label = widgets.Label(value="Offset Output: 0")
        self.status_label = widgets.Label(value="Status: Ingen linjer funnet")
        self.height_label1 = widgets.Label(value="Høyde 1: ")
        self.offset_label1 = widgets.Label(value="Offset 1: Venstre , Høyre ")
        self.height_label2 = widgets.Label(value="Høyde 1: ")
        self.offset_label2 = widgets.Label(value="Offset 1: Venstre , Høyre ")

        self.kp_slider = widgets.FloatSlider(min=0.0, max=1, step=0.01, value=0.2, description='Kp')
        self.ki_slider = widgets.FloatSlider(min=0.0, max=1, step=0.001, value=0.000, description='Ki')
        self.kd_slider = widgets.FloatSlider(min=0.0, max=1, step=0.01, value=0.3, description='Kd')
        self.speed_slider = widgets.FloatSlider(min=0.0, max=1.0, step=0.01, value=0, description='Speed')

        layout = widgets.VBox([
            widgets.HBox([
                widgets.VBox([self.lines_widget]),
                widgets.VBox([self.kp_slider, self.ki_slider, self.kd_slider, self.speed_slider]),
                widgets.VBox(
                    [self.error_label, self.output_label, self.angle_label, self.offset_label, self.status_label]),
                widgets.VBox(
                    [self.height_label1, self.offset_label1, self.height_label2, self.offset_label2])
            ])
        ])
        display(layout)

        self.last_seen_two_lines = time.time()
        self.last_seen_lines = None

        self.mask = None
        self.line_image = None

        self.prev_error = 0
        self.integral = 0

        self.camera.observe(self.update_widget, names='value')
        print("Track Follower initialized and ready.")

    def convert_to_image_widget(self, frame):
        _, encoded_image = cv2.imencode('.jpg', frame)
        return encoded_image.tobytes()

    def detect_edges(self, frame):
        mask = cv2.inRange(frame, self.lower_bound, self.upper_bound)
        blur = cv2.GaussianBlur(mask, (7, 7), 0)
        edges = cv2.Canny(blur, 50, 200)

        return edges

    def region_of_interest(self, edges, region_height_fraction=0.7):
        if self.mask is None:
            height, width = edges.shape
            mask = np.zeros_like(edges)
            polygon = np.array([[
                (0, height),
                (width, height),
                (width, int(height * (1 - region_height_fraction))),
                (0, int(height * (1 - region_height_fraction))),
            ]], np.int32)
            cv2.fillPoly(mask, polygon, 255)
            self.mask = mask
        return cv2.bitwise_and(edges, self.mask)

    def scan_for_edges(self, image, start_row=0, end_row=None):
        height, width = image.shape
        if end_row is None:
            end_row = height

        current_center = width // 2
        left_points, right_points, midline_points = [], [], []
        prev_left, prev_right = None, None
        min_distance = 32
        max_distance = 112

        for row in range(end_row - 1, start_row - 1, -1):
            row_pixels = image[row]

            left_candidates = np.flatnonzero(row_pixels[:current_center] > 254)
            right_candidates = np.flatnonzero(row_pixels[current_center:] > 254)

            left_edge = left_candidates[-1] if left_candidates.size else None
            right_edge = right_candidates[0] + current_center if right_candidates.size else None

            if left_edge is not None and right_edge is not None:
                distance = right_edge - left_edge
                if left_edge < right_edge and min_distance <= distance <= max_distance:
                    mid_x = (left_edge + right_edge) / 2
                    current_center = int(mid_x)
                    left_points.append((left_edge, row))
                    right_points.append((right_edge, row))
                    prev_left, prev_right = left_edge, right_edge

            elif left_edge is not None:
                if prev_left is not None:
                    current_center += (left_edge - prev_left)
                left_points.append((left_edge, row))
                prev_left = left_edge

            elif right_edge is not None:
                if prev_right is not None:
                    current_center += (right_edge - prev_right)
                right_points.append((right_edge, row))
                prev_right = right_edge

            else:
                if prev_left is not None and prev_right is not None:
                    current_center = (prev_left + prev_right) // 2

        left_points = self.fit_and_filter_points(left_points)
        right_points = self.fit_and_filter_points(right_points)

        left_dict = {row: x for x, row in left_points}
        right_dict = {row: x for x, row in right_points}

        all_rows = sorted(set(left_dict.keys()).union(right_dict.keys()))

        for row in all_rows:
            left_x = left_dict.get(row)
            right_x = right_dict.get(row)

            if left_x is not None and right_x is not None:
                mid_x = (left_x + right_x) / 2
                midline_points.append((mid_x, row))

        return left_points, right_points, midline_points

    def fit_and_filter_points(self, points, poly_degree=5, threshold=5):
        if len(points) < poly_degree + 1:
            return points

        rows = np.array([point[1] for point in points])
        x_values = np.array([point[0] for point in points])

        poly_coeffs = np.polyfit(rows, x_values, poly_degree)
        poly_func = np.poly1d(poly_coeffs)

        predicted_x = poly_func(rows)

        filtered_points = [
            (x, row) for x, row, pred_x in zip(x_values, rows, predicted_x)
            if abs(x - pred_x) <= threshold
        ]

        return filtered_points

    def find_common_navigation_points(self, midline_points, left_points, right_points):
        if not midline_points:
            return None, None

        midline_heights = [point[1] for point in midline_points]

        height_counts = Counter(midline_heights)
        most_common_heights = height_counts.most_common(2)

        if len(most_common_heights) < 2:
            return None, None

        common_height1 = most_common_heights[0][0]
        common_height2 = most_common_heights[1][0]

        offsets1 = []
        offsets2 = []

        for mid, left, right in zip(midline_points, left_points, right_points):
            if mid[1] == common_height1:
                offsets1.append((mid[0] - left[0], right[0] - mid[0]))
            elif mid[1] == common_height2:
                offsets2.append((mid[0] - left[0], right[0] - mid[0]))

        avg_offset1 = (
            sum(offset[0] for offset in offsets1) / len(offsets1) if offsets1 else 0,
            sum(offset[1] for offset in offsets1) / len(offsets1) if offsets1 else 0
        )
        avg_offset2 = (
            sum(offset[0] for offset in offsets2) / len(offsets2) if offsets2 else 0,
            sum(offset[1] for offset in offsets2) / len(offsets2) if offsets2 else 0
        )

        self.height_label1.value = f"Høyde 1: {common_height1}"
        self.offset_label1.value = f"Offset 1: Venstre {avg_offset1[0]:.2f}, Høyre {avg_offset1[1]:.2f}"

        self.height_label2.value = f"Høyde 2: {common_height2}"
        self.offset_label2.value = f"Offset 2: Venstre {avg_offset2[0]:.2f}, Høyre {avg_offset2[1]:.2f}"

        return (common_height1, avg_offset1), (common_height2, avg_offset2)

    def calculate_error(self, midline_points, frame_width):
        if len(midline_points) < 2:
            return 0

        x1, y1 = midline_points[-1][0], midline_points[-1][1]
        x2, y2 = midline_points[-2][0], midline_points[-2][1]

        angle_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))
        adjusted_angle = -(angle_deg + 90)

        mid_x = frame_width // 2
        offset = mid_x - x1

        normalized_offset = offset / (frame_width // 2)
        normalized_angle = adjusted_angle / 90.0

        self.angle_label.value = f"Angle Output: {adjusted_angle:.2f}"
        self.offset_label.value = f"Offset Output: {normalized_offset:.2f}"

        weight_offset = 1.0
        weight_angle = 0.1

        combined_error = (weight_offset * normalized_offset) + (weight_angle * normalized_angle)
        return combined_error

    def process_image(self, frame):
        edges = self.detect_edges(frame)
        roi = self.region_of_interest(edges, region_height_fraction=0.75)
        height = frame.shape[0]
        width = frame.shape[1]
        start_row = int(height * 0.25)
        end_row = int(height * 1)
        target_heights_offsets = [(83, 53), (82, 52)]

        left_points, right_points, midline_points = self.scan_for_edges(roi, start_row=start_row, end_row=end_row)
        current_time = time.time()

        # self.find_common_navigation_points(midline_points, left_points, right_points)

        if self.line_image is None:
            self.line_image = np.zeros_like(frame)

        self.line_image.fill(0)

        for points, color in [(left_points, (255, 0, 0)), (right_points, (0, 255, 0))]:
            for point in points:
                cv2.circle(self.line_image, (int(point[0]), int(point[1])), 2, color, -1)

        if midline_points:
            self.last_seen_two_lines = current_time
            self.status_label.value = "Status: To linjer funnet"
            self.last_seen_lines = "two"
            error = self.calculate_error(midline_points, width)

            for pt1, pt2 in zip(midline_points[:-1], midline_points[1:]):
                cv2.line(self.line_image, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 0, 255), 2)
        else:
            time_since_last_seen = current_time - self.last_seen_two_lines
            if time_since_last_seen < 2.0:
                self.status_label.value = "Status: Venter på to linjer"
                error = self.prev_error
            if left_points and not right_points:
                self.status_label.value = "Status: Venstre linje funnet"
                if self.last_seen_lines != "left_only":
                    self.last_seen_lines = "left_only"

                estimated_midline_points = []
                for target_height, target_offset in target_heights_offsets:
                    left_point = next((p for p in left_points if p[1] == target_height), None)
                    if left_point:
                        mid_x = left_point[0] + target_offset
                        estimated_midline_points.append((mid_x, target_height))

                if len(estimated_midline_points) == len(target_heights_offsets):
                    error = self.calculate_error(estimated_midline_points, width)
                    for pt1, pt2 in zip(estimated_midline_points[:-1], estimated_midline_points[1:]):
                        cv2.line(self.line_image, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 0, 255),
                                 2)
                else:
                    error = self.prev_error
            elif right_points and not left_points:
                self.status_label.value = "Status: Høyre linje funnet"
                if self.last_seen_lines != "right_only":
                    self.last_seen_lines = "right_only"

                estimated_midline_points = []
                for target_height, target_offset in target_heights_offsets:
                    right_point = next((p for p in right_points if p[1] == target_height), None)
                    if right_point:
                        mid_x = right_point[0] - target_offset
                        estimated_midline_points.append((mid_x, target_height))

                if len(estimated_midline_points) == len(target_heights_offsets):
                    error = self.calculate_error(estimated_midline_points, width)
                    for pt1, pt2 in zip(estimated_midline_points[:-1], estimated_midline_points[1:]):
                        cv2.line(self.line_image, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 0, 255),
                                 2)
                else:
                    error = self.prev_error
            else:
                self.status_label.value = "Status: Ingen linjer funnet"
                if self.last_seen_lines != "none":
                    error = self.prev_error
                else:
                    self.robot.stop()
                    return

        self.follow_line(error, len(midline_points))

        combined_image = cv2.addWeighted(frame, 0.8, self.line_image, 1, 1)
        self.lines_widget.value = self.convert_to_image_widget(combined_image)
        return combined_image

    def follow_line(self, error, num_midline_points):
        min_speed = 0.09
        max_speed = 0.25
        max_points = 30

        num_midline_points = min(num_midline_points, max_points)
        base_speed = min_speed + (max_speed - min_speed) * (num_midline_points / max_points)

        pid_output = self.pid_control(error)

        self.error_label.value = f"Feil: {error:.2f}"
        self.output_label.value = f"PID Output: {pid_output:.2f}"
        self.speed_slider.value = base_speed

        max_speed = 1

        left_speed = max(min(base_speed - pid_output, max_speed), 0)
        right_speed = max(min(base_speed + pid_output, max_speed), 0)

        self.robot.set_motors(left_speed, right_speed)

    def pid_control(self, error):
        kp = self.kp_slider.value
        ki = self.ki_slider.value
        kd = self.kd_slider.value

        alpha = 0.8
        filtered_error = alpha * self.prev_error + (1 - alpha) * error

        p = kp * filtered_error
        self.integral += filtered_error
        i = ki * self.integral
        d = kd * (filtered_error - self.prev_error)

        self.prev_error = filtered_error

        return p + i + d

    def update_widget(self, change):
        frame = change['new']
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        processed_frame = self.process_image(frame)
        if processed_frame is not None:
            self.heading_widget.value = self.convert_to_image_widget(processed_frame)
        else:
            print("Processed frame is None, skipping update.")
            self.robot.stop()

    def stop(self):
        self.robot.stop()
        self.camera.unobserve_all()
        self.camera.close()
        print("Track Follower stopped and resources released.")
