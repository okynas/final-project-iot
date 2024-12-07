import unittest
from Robot.Manager.ConfigManager import ConfigManager


class TemplateConfig:
    id = 4
    robot_id = "Rask-Rut"


class ConfigManagerTests(unittest.TestCase):
    def setUp(self):
        self.config_manager = ConfigManager(debug=False)

    def tearDown(self):
        del self.config_manager

    def test_read_config(self):
        self.assertIsNotNone(self.config_manager)

    def test_id_is_set(self):
        self.assertNotEqual(self.config_manager.id, 0)

    def test_default_values(self):
        self.assertNotEqual(self.config_manager.default_handler.default_distance, 0)
        self.assertNotEqual(self.config_manager.default_handler.default_speed, 0)

    def test_read_mqtt_values(self):
        self.assertNotEqual(self.config_manager.mqtt_handler.mqtt_ip, "")
        self.assertNotEqual(self.config_manager.mqtt_handler.mqtt_port, 0)

    def test_dynamic_robot(self):
        self.assertNotEqual(self.config_manager.robot.robot_id, 0)
        self.assertEqual(self.config_manager.robot.id, TemplateConfig.id)
        self.assertEqual(self.config_manager.robot.robot_id, TemplateConfig.robot_id)

if __name__ == '__main__':
    unittest.main()
