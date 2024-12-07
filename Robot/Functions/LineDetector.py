import cv2
import numpy as np
import logging

class Detector:
    def __init__(self):
        self.blur_size = (3,3)

        self.threshold = 30
        self.min_line_length = 70
        self.max_line_gap = 30

        # Oppdater Canny values
        self.canny_low = 50
        self.canny_high = 150

    def get_blur(self, image):
        return cv2.GaussianBlur(image, self.blur_size, 0)

    def get_canny(self, image):
        return cv2.Canny(image, self.canny_low, self.canny_high)

class LineDetector:
    def __init__(self, camera, width=320, height=240, roi=0.6):
        self.camera = camera
        self.image_height = height
        self.image_width = width
        self.roi = roi
        self.roi_height = int(self.image_height * (1 - self.roi))
        self.previous_steering_angle = None

        self.detector = Detector()

        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

    def preprocess_image(self, image):
        try:
            roi = image[self.roi_height:, :]

            gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)

            # Perform edge detection
            mask = self.detector.get_blur(gray)
            mask_cleaned = self.detector.get_canny(mask)

            # mask_cleaned = mask

            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_cleaned, connectivity=8)
            areas = stats[1:, cv2.CC_STAT_AREA]

            road = "big" if len(areas) == 0 or np.mean(areas) <= 70 else "small"

            if road == "small":
                min_area = np.mean(areas) * 0.8
                mask_cleaned = self.filter_areas(mask_cleaned, labels, stats, min_area)

            return roi, mask, mask_cleaned, road
        except Exception as e:
            logging.error(f"Feil under preprocess_image: {e}")
            raise

    def filter_areas(self, roi, labels, stats, min_area):
        try:
            filtered_binary = np.zeros_like(roi)
            for label, area in enumerate(stats[1:, cv2.CC_STAT_AREA], start=1):
                if area >= min_area:
                    filtered_binary[labels == label] = 255
            return filtered_binary
        except Exception as e:
            logging.error(f"Feil under filter_areas: {e}")
            raise

    def apply_morphological_operations(self, binary_image, kernel_size=3):
        try:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
            binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
            return binary_image

        except Exception as e:
            logging.error(f"Feil under apply_morphological_operations: {e}")
            raise

    def detect_lines(self, binary_image, road):
        roi = binary_image

        if road == "small":
            hough_status = "small road hough"
        else:
            hough_status = "big road hough"

        lines = self.detect_hough_lines(binary_image, road)

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / (x2 - x1)

                line_center_x = (x1 + x2) / 2
                if slope < 0 and line_center_x < self.image_width / 2:
                    left_lines.append(line[0])
                elif slope > 0 and line_center_x > self.image_width / 2:
                    right_lines.append(line[0])

        return left_lines, right_lines, roi, hough_status

    def detect_hough_lines(self, roi, road_type):
        try:
            params = {
                "small": {"threshold": self.detector.threshold, "minLineLength": self.detector.min_line_length, "maxLineGap": self.detector.max_line_gap},
                "big": {"threshold": 20, "minLineLength": 30, "maxLineGap": 50}
            }
            config = params[road_type]
            return cv2.HoughLinesP(
                roi,
                rho=1,
                theta=np.pi / 180,
                threshold=config["threshold"],
                minLineLength=config["minLineLength"],
                maxLineGap=config["maxLineGap"]
            )
        except Exception as e:
            logging.error(f"Feil under detect_hough_lines: {e}")
            raise

    def calculate_steering_angle(self, left_lines, right_lines, road):
        image_center = self.image_width // 2

        if not hasattr(self, 'previous_steering_angle'):
            self.previous_steering_angle = 0

        if not left_lines and not right_lines:
            return self._handle_no_lines_found(image_center)

        if left_lines and right_lines:
            return self._handle_both_lines(left_lines, right_lines, image_center)

        active_lines = right_lines if right_lines else left_lines
        return self._handle_single_line(active_lines, image_center, bool(right_lines))

    def _handle_no_lines_found(self, image_center):
        status_message = "Ingen linjer funnet. Beholder forrige styrevinkel."
        return self.previous_steering_angle, image_center, image_center, 0, None, status_message

    def _handle_both_lines(self, left_lines, right_lines, image_center):
        left_bottom = self.get_bottom_point(left_lines)
        right_bottom = self.get_bottom_point(right_lines)

        if left_bottom is None or right_bottom is None:
            return self._handle_no_lines_found(image_center)

        center_x = (left_bottom[0] + right_bottom[0]) // 2
        deviation = center_x - image_center
        dx = right_bottom[0] - left_bottom[0]
        dy = right_bottom[1] - left_bottom[1]
        angle = np.arctan2(dy, dx) * 180 / np.pi

        steering_angle = self._calculate_weighted_angle(angle, deviation)

        bottom_points = {
            'left': (left_bottom[0], left_bottom[1] + self.roi_height),
            'right': (right_bottom[0], right_bottom[1] + self.roi_height),
            'angle': angle,
            'single_line': False
        }

        status_message = "Begge linjer funnet."
        self.previous_steering_angle = steering_angle
        return steering_angle, center_x, image_center, deviation, bottom_points, status_message

    def _handle_single_line(self, lines, image_center, using_right_line):
        bottom_point = self.get_bottom_point(lines)
        top_point = self.get_top_point(lines)

        if bottom_point is None or top_point is None:
            return self._handle_no_lines_found(image_center)

        angle, target_angle, vertical_factor, distance_to_center = self.calculate_target_angle(
            bottom_point, top_point, using_right_line
        )
        steering_angle = np.clip(-(angle - target_angle) * 0.5, -30, 30)

        deviation = bottom_point[0] - image_center
        bottom_points = {
            'single_line': True,
            'line_angle': angle,
            'bottom_point': (bottom_point[0], bottom_point[1] + self.roi_height),
            'top_point': (top_point[0], top_point[1] + self.roi_height),
            'using_right_line': using_right_line,
            'target_angle': target_angle,
            'distance_to_center': distance_to_center,
            'vertical_factor': vertical_factor,
            'vertical_position': (self.image_height / 2) - bottom_point[1]
        }

        status_message = "En linje funnet."
        self.previous_steering_angle = steering_angle
        return steering_angle, bottom_point[0], image_center, deviation, bottom_points, status_message

    def _calculate_weighted_angle(self, angle, deviation):
        position_weight = 0.5
        angle_weight = 0.5
        max_angle = 30
        position_factor = deviation / (self.image_width / 2)
        angle_factor = angle / max_angle
        return -(position_weight * position_factor + angle_weight * angle_factor) * max_angle

    def calculate_target_angle(self, bottom_point, top_point, using_right_line):
        dx = top_point[0] - bottom_point[0]
        dy = top_point[1] - bottom_point[1]
        angle = np.arctan2(dx, dy) * 180 / np.pi

        distance_to_center = abs(bottom_point[0] - (self.image_width // 2))
        max_distance = self.image_width / 2
        max_vertical = self.image_height / 4

        vertical_position = (self.image_height / 2) - bottom_point[1]
        vertical_factor = vertical_position / max_vertical

        if vertical_factor > 0.1:
            target_angle = -120 - (25 * (1 - vertical_factor)) if using_right_line else 120 + (
                    25 * (1 - vertical_factor))
        else:
            distance_factor = distance_to_center / max_distance
            target_angle = (-190 + (45 * distance_factor)) if using_right_line else (190 - (45 * distance_factor))

        return angle, target_angle, vertical_factor, distance_to_center

    def get_bottom_point(self, lines):
        lines = np.array(lines).reshape(-1, 4)
        max_y_idx = np.argmax(lines[:, [1, 3]].max(axis=1))
        return (lines[max_y_idx, 2], lines[max_y_idx, 3]) if lines[max_y_idx, 3] > lines[max_y_idx, 1] else \
            (lines[max_y_idx, 0], lines[max_y_idx, 1])

    def get_top_point(self, lines):
        lines = np.array(lines).reshape(-1, 4)
        min_y_idx = np.argmin(lines[:, [1, 3]].min(axis=1))
        return (lines[min_y_idx, 2], lines[min_y_idx, 3]) if lines[min_y_idx, 3] < lines[min_y_idx, 1] else \
            (lines[min_y_idx, 0], lines[min_y_idx, 1])

    def draw_lines(self, image, lines, color):
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(image, (x1, y1 + self.roi_height), (x2, y2 + self.roi_height), color, 2)

    def process_frame(self, visualize=True):
        try:
            image = self.camera.value
            if image is None:
                logging.warning("Ingen bilde fra kameraet")
                return None

            if image.shape[:2] != (self.image_height, self.image_width):
                image = cv2.resize(image, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)
            roi, masked, masked_cleand, road = self.preprocess_image(image)

            left_lines, right_lines, roi, hough_status = self.detect_lines(masked_cleand, road)
            steering_angle, center_x, image_center, deviation, bottom_points, status_message = self.calculate_steering_angle(
                left_lines, right_lines, road)

            visualization = image.copy()
            if visualize:

                if left_lines or right_lines:
                    self.draw_lines(visualization, left_lines, (0, 0, 255))
                    self.draw_lines(visualization, right_lines, (0, 255, 0))

                    if bottom_points:
                        if bottom_points.get('single_line', False):
                            bottom = bottom_points['bottom_point']
                            top = bottom_points['top_point']

                            cv2.circle(visualization, bottom, 5, (255, 255, 0), -1)
                            cv2.circle(visualization, top, 5, (255, 255, 0), -1)
                            cv2.line(visualization, bottom, top, (0, 255, 255), 2)
                        else:
                            left_bottom = bottom_points['left']
                            right_bottom = bottom_points['right']

                            cv2.circle(visualization, left_bottom, 5, (255, 255, 0), -1)
                            cv2.circle(visualization, right_bottom, 5, (255, 255, 0), -1)
                            cv2.line(visualization, left_bottom, right_bottom, (255, 0, 0), 2)
                pass

            mode = "Single Line" if bottom_points and bottom_points.get('single_line') else "Two Lines"

            angle = bottom_points.get('line_angle', 0) if bottom_points else 0
            hough_status = hough_status if hough_status else "Unknown"

            return {
                'steering_angle': steering_angle,
                'mode': mode,
                'angle': angle,
                'hough_transform': hough_status,
                'processed_images': {
                    'original': cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                    'masked': masked,
                    'masked_cleand': masked_cleand,
                    'roi': roi,
                    'result': cv2.cvtColor(visualization, cv2.COLOR_BGR2RGB)
                },
                'status_message': status_message
            }

        except Exception as e:
            logging.error(f"Feil under process_frame: {e}")
            raise

    def stop(self):
        logging.info("Stopper LineDetector")
        self.camera.stop()