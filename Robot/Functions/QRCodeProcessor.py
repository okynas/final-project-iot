from pyzbar.pyzbar import decode
import cv2

class QRCodeProcessor:
    def __init__(self, camera):
        self.camera = camera

    def detect_qr_code(self):
        """Fanger bilde fra kameraet og dekoder QR-koder."""
        frame = self.camera.value
        if frame is None:
            return None

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced_frame = cv2.equalizeHist(gray_frame)

        decoded_objects = decode(enhanced_frame)

        if decoded_objects:
            qr_data = decoded_objects[0].data.decode('utf-8')
            return qr_data

        return None
