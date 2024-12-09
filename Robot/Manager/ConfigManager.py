import os
import toml

class ConfigManager:
    """
    A class to manage configuration for a robot, including handling, MQTT, and robot-specific settings.
    """

    class HandlingConfig:
        """
        Represents default handling settings for the robot.
        """
        def __init__(self):
            self.default_distance = 1  # Default distance setting for the robot's movement
            self.default_speed = 1     # Default speed setting for the robot

    class MqttConfig:
        """
        Represents MQTT (Message Queuing Telemetry Transport) configuration.
        """
        def __init__(self):
            self.mqtt_ip = ""          # IP address of the MQTT broker
            self.mqtt_port = 0         # Port number for the MQTT broker

    class Robot:
        """
        Represents robot-specific configuration.
        """
        def __init__(self):
            self.id = 0               # Identifier for the robot
            self.robot_id = ""        # Unique robot ID (as per configuration)

    def log(self, message):
        """Logger en melding via log_callback."""
        self.log_callback(message)

    def __init__(self, debug=True, filename='config.toml', log_callback=None):
        """
        Initializes the ConfigManager, setting up paths and loading configuration.
        """

        self.debug = debug
        self.log_callback = log_callback if log_callback else print

        # Path to the configuration file (relative to the script's directory)
        self._config_file_path = os.path.join(os.path.dirname(__file__), '..', filename)

        # Handlers for different configuration sections
        self.default_handler = None  # Default handling configuration
        self.mqtt_handler = None     # MQTT configuration
        self.robot = None            # Robot-specific configuration
        self.id = 0                  # Current robot ID

        # Load the configuration file during initialization
        self._load_config()

    def _read_jetbot(self, id: int, toml_file: dict) -> Robot:
        """
        Reads and loads the configuration for a specific robot (JetBot) from the TOML file.

        :param id: ID of the robot to load
        :param toml_file: Parsed TOML file as a dictionary
        :return: Robot object with populated data
        """
        jetbot = self.Robot()
        jetbot.id = id
        # Extract robot ID, defaulting to an empty string if not found
        jetbot.robot_id = toml_file[f"jetbot-{id}"]['robot_id'] or ""
        return jetbot

    def _read_mqtt(self, toml_file: dict) -> MqttConfig:
        """
        Reads and loads the MQTT configuration from the TOML file.

        :param toml_file: Parsed TOML file as a dictionary
        :return: MqttConfig object with populated data
        """
        mqtt = self.MqttConfig()
        # Extract MQTT IP and port, defaulting to empty and 0 if not found
        mqtt.mqtt_ip = toml_file['mqtt']['ip'] or ""
        mqtt.mqtt_port = toml_file['mqtt']['port'] or 0
        return mqtt

    def _read_default_handling(self, toml_file: dict) -> HandlingConfig:
        """
        Reads and loads the default handling configuration from the TOML file.

        :param toml_file: Parsed TOML file as a dictionary
        :return: HandlingConfig object with populated data
        """
        handler = self.HandlingConfig()
        # Extract speed and distance settings, defaulting to 0 if not found
        handler.default_speed = toml_file['default-handling']['speed'] or 0
        handler.default_distance = toml_file['default-handling']['distance'] or 0
        return handler

    def _load_config(self):
        """
        Loads the TOML configuration file and populates internal attributes.
        """
        # Check if the configuration file exists
        if not os.path.exists(self._config_file_path):
            raise FileNotFoundError(f"Config file not found: {self._config_file_path}")

        # Open and parse the TOML configuration file
        with open(self._config_file_path, 'r') as file:
            toml_file = toml.load(file)
            if self.debug: self.log(f"Read config OK")

        # Load current JetBot ID from the configuration
        self.id = toml_file['current-jetbot']['id']
        # Load robot-specific, handling, and MQTT configurations
        self.robot = self._read_jetbot(self.id, toml_file)
        self.default_handler = self._read_default_handling(toml_file)
        self.mqtt_handler = self._read_mqtt(toml_file)
