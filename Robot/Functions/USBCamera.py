import cv2
from threading import Thread
import ipywidgets.widgets as widgets
from IPython.display import display

class USBCamera:
    _instance = None

    def __init__(self, width=640, height=480, fps=30, pixel_format='MJPG', device='/dev/video0'):
        self.width = width
        self.height = height
        self.fps = fps
        self.device = device

        # Sett opp VideoCapture med riktig format og oppløsning
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))

        if not self.cap.isOpened():
            raise RuntimeError(f"Kunne ikke åpne kameraet på {self.device}")

        self.running = True
        self.frame = None
        self.thread = Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    @classmethod
    def instance(cls, width=640, height=480, fps=30, pixel_format='MJPG', device='/dev/video0'):
        if cls._instance is None:
            cls._instance = cls(width, height, fps, pixel_format, device)
        return cls._instance

    @property
    def value(self):
        return self.frame

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()

class CameraWidget:
    def __init__(self, camera: USBCamera):
        self.camera = camera
        self.image_widget = widgets.Image(format='jpeg', width=self.camera.width // 2, height=self.camera.height // 2)
        display(self.image_widget)
        self.running = True
        self.thread = Thread(target=self._update_widget, daemon=True)
        self.thread.start()

    def _update_widget(self):
        while self.running:
            frame = self.camera.value
            if frame is not None:
                _, jpeg = cv2.imencode('.jpg', frame)
                self.image_widget.value = jpeg.tobytes()

    def stop(self):
        self.running = False
        self.thread.join()
