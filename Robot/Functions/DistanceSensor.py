import VL53L1X
import logging

logger = logging.getLogger("DistanceSensor")
logging.basicConfig(level=logging.INFO)

class DistanceSensor:
    def __init__(self, i2c_bus=1, i2c_address=0x29, ranging_mode=1):
        self.tof = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=i2c_address)
        self.tof.open()
        self.ranging_mode = ranging_mode
        self.tof.start_ranging(self.ranging_mode)
        logger.info(f"VL53L1X sensor initialisert på I2C-bus {i2c_bus} med adresse {hex(i2c_address)}")

    def get_distance(self):
        """
        Returnerer avstanden målt av VL53L1X-sensoren i millimeter.
        :return: Avstand i mm, eller None hvis en feil oppstår.
        """
        try:
            distance_in_mm = self.tof.get_distance()
            return distance_in_mm
        except Exception as e:
            logger.info(f"Feil ved avlesning fra VL53L1X: {e}")
            return None

    def stop_sensor(self):
        """
        Stopper VL53L1X-sensoren.
        """
        self.tof.stop_ranging()
        logger.info("VL53L1X sensor stoppet.")
