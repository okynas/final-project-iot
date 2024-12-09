import time
from Functions.QRCodeProcessor import QRCodeProcessor

class QRCodeManager:
    def __init__(self, camera, platoon_manager, distance_manager, log_callback=None, timeout=2.0):
        """
        Håndterer QR kode platooning.
        """
        self.qr_processor = QRCodeProcessor(camera)
        self.platoon_manager = platoon_manager
        self.distance_manager = distance_manager
        self.log_callback = log_callback if log_callback else print
        self.timeout = timeout
        self.last_qr_data = None
        self.last_update_time = time.time()

    def log(self, message):
        """Logger melding hvis log_callback er satt."""
        if self.log_callback:
            self.log_callback(message)

    def process_qr_codes(self):
        """Les og håndter QR-koder kontinuerlig."""
        while True:
            qr_data = self.qr_processor.detect_qr_code()
            current_time = time.time()

            if qr_data:
                if qr_data != self.last_qr_data:
                    self.platoon_manager.front_robot_id = qr_data
                    self.platoon_manager.join_platoon(qr_data)
                    self.log(f"QR-kode funnet og identifisert som: {qr_data}")
                    self.last_qr_data = qr_data
                self.last_update_time = current_time
            else:
                if current_time - self.last_update_time > self.timeout:
                    if self.last_qr_data is not None:
                        self.log("Bilen foran har kjørt vekk!")
                    self.platoon_manager.leave_platoon()
                    self.platoon_manager.front_robot_id = None
                    self.last_qr_data = None

            time.sleep(0.05)
