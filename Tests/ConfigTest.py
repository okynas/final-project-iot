import unittest
from Manager.ConfigManager import ConfigManager


class ConfigManagerTests(unittest.TestCase):
    def test_read_config(self):
        config_manager = ConfigManager()

        self.assertNotEqual(config_manager.id, 0)
        self.assertNotEqual(config_manager.default_handler.default_distance, 0)
        self.assertNotEqual(config_manager.default_handler.default_speed, 0)

        self.assertNotEqual(config_manager.mqtt_handler.mqtt_ip, "")
        self.assertNotEqual(config_manager.mqtt_handler.mqtt_port, 0)



if __name__ == '__main__':
    unittest.main()
