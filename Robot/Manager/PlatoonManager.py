import json
import threading
import time
import paho.mqtt.client as mqtt
from Manager.RoadFollower import RobotState

class PlatoonManager:
    def __init__(self, robot_id, mqtt_broker, port, road_follower, distance_manager, log_callback=None):
        """
        Dynamisk håndtering av platooning.
        """
        self.robot_id = robot_id
        self.mqtt_client = mqtt.Client(robot_id)
        self.mqtt_broker = mqtt_broker
        self.port = port
        self.road_follower = road_follower
        self.distance_manager = distance_manager
        self.log_callback = log_callback if log_callback else print

        self.is_leader = road_follower.is_leader
        self.queue_position = None
        self.front_robot_id = None
        self.followers = []
        self.platoon_speed = road_follower.base_speed
        self.steering_angle = 0
        self.last_heartbeat = time.time()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

    def log(self, message):
        """Logger meldinger via callback."""
        self.log_callback(message)

    def determine_leader(self):
        """Bestem leder basert på front_robot_id."""
        if not self.front_robot_id:
            if not self.is_leader:
                self.log(f"Robot {self.robot_id} har blitt lederen!")
            self.is_leader = True
            self.queue_position = 0
        else:
            if self.is_leader:
                self.log(f"Robot {self.robot_id} er ikke lenger lederen.")
            self.is_leader = False

        self.road_follower.set_is_leader(self.is_leader)

    def join_platoon(self, front_robot_id):
        """Legg roboten til som en følger bak front_robot_id."""
        self.front_robot_id = front_robot_id
        self.determine_leader()
        if not self.is_leader:
            self.subscribe_to_front_robot()
            self.log(f"Robot {self.robot_id} har sluttet seg til køen bak {self.front_robot_id}.")
        else:
            self.log(f"Robot {self.robot_id} er allerede leder, ingen behov for å bli med i køen.")

    def leave_platoon(self):
        """Fjern roboten fra platoonen."""
        if self.front_robot_id:
            self.unsubscribe_from_front_robot()
            self.log(f"Robot {self.robot_id} har forlatt køen bak {self.front_robot_id}.")
        self.front_robot_id = None
        self.queue_position = None
        self.is_leader = False

    def subscribe_to_front_robot(self):
        """Abonner på statusmeldinger fra bilen foran."""
        if self.front_robot_id:
            topic = f"platoon/status/{self.front_robot_id}"
            self.mqtt_client.subscribe(topic)
            self.log(f"Robot {self.robot_id} abonnerer nå på {topic}.")

    def unsubscribe_from_front_robot(self):
        """Avslutter abonnement på statusmeldinger fra bilen foran."""
        if self.front_robot_id:
            topic = f"platoon/status/{self.front_robot_id}"
            self.mqtt_client.unsubscribe(topic)
            self.log(f"Robot {self.robot_id} avslutter abonnement på {topic}.")

    def update_platoon_structure(self, data):
        """Oppdater strukturen i platoonen basert på meldinger."""
        if "followers" in data:
            updated_followers = list(dict.fromkeys(data["followers"]))
            self.followers = updated_followers
            self.log(f"Robot {self.robot_id} har følgende oppdaterte kø: {self.followers}")

            if self.robot_id in self.followers:
                self.queue_position = self.followers.index(self.robot_id)
                self.log(f"Robot {self.robot_id} sin posisjon i køen: {self.queue_position}")

    def handle_dynamic_commands(self, data):
        """Håndterer kommandoer som 'join_queue' og 'leave_queue'."""
        command = data.get("command")
        if command == "set_speed":
            new_speed = data.get("speed")
            if new_speed is not None:
                self.distance_manager.update_platoon_speed(new_speed)
                self.log(f"Robot {self.robot_id} oppdaterte sin fart til {self.platoon_speed:.2f}")
        elif command == "change_status":
            new_state = data.get("new_status")
            if new_state and new_state in RobotState.__members__:
                self.road_follower.state = RobotState[new_state]
                self.log(f"Robot {self.robot_id} oppdaterte sin status til {new_state}")
        elif command == "join_queue":
            new_follower_id = data.get("robot_id")
            if new_follower_id not in self.followers:
                self.followers.append(new_follower_id)
                self.log(f"Robot {new_follower_id} har sluttet seg til køen.")
        elif command == "leave_queue":
            leaving_robot_id = data.get("robot_id")
            if leaving_robot_id in self.followers:
                self.followers.remove(leaving_robot_id)
                self.log(f"Robot {leaving_robot_id} har forlatt køen.")

    def on_connect(self, client, userdata, flags, rc):
        """Callback når vi kobler til MQTT-broker."""
        self.log(f"Robot {self.robot_id} koblet til MQTT Broker")
        self.mqtt_client.subscribe("platoon/commands")
        self.mqtt_client.subscribe(f"platoon/commands/{self.robot_id}")
        self.mqtt_client.subscribe(f"platoon/status/#")

    def on_message(self, client, userdata, msg):
        """Behandle meldinger fra MQTT."""
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if msg.topic == f"platoon/commands/{self.robot_id}" or msg.topic == "platoon/commands":
                self.handle_dynamic_commands(data)
            elif msg.topic.startswith(f"platoon/status/{self.front_robot_id}"):
                if "platoon_speed" in data:
                    self.distance_manager.update_platoon_speed(data["platoon_speed"])
                    self.log(
                        f"Robot {self.robot_id} oppdaterte sin platoon_speed fra {self.front_robot_id}: {data['platoon_speed']:.2f}")

                if "state" in data:
                    new_state = data["state"]
                    if new_state != self.road_follower.state.name:
                        self.road_follower.state = RobotState[new_state]
                        self.log(
                            f"Robot {self.robot_id} oppdaterte sin state til {new_state} basert på {self.front_robot_id}")

                if "steering_angle" in data:
                    self.road_follower.set_steering_angle(data["steering_angle"], self.distance_manager.distance)

            if msg.topic.startswith("platoon/status/"):
                self.update_platoon_structure(data)
        except Exception as e:
            self.log(f"Feil i on_message: {e}")

    def publish_status(self):
        """Publiser robotens status til MQTT-brokeren."""
        while True:
            self.determine_leader()

            self.steering_angle = self.road_follower.steering_angle

            status = {
                "robot_id": self.robot_id,
                "queue_position": self.queue_position,
                "role": "leader" if self.is_leader else "follower",
                "state": self.road_follower.state.name,
                "platoon_speed": self.platoon_speed,
                "front_robot_id": self.front_robot_id,
                "steering_angle": self.steering_angle,
                "followers": self.followers,
                "last_heartbeat": time.time()
            }
            self.mqtt_client.publish(f"platoon/status/{self.robot_id}", json.dumps(status))
            time.sleep(0.5)

    def start(self):
        """Start MQTT-klienten og statuspublisering."""
        self.mqtt_client.connect(self.mqtt_broker, self.port, 60)
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
        threading.Thread(target=self.publish_status, daemon=True).start()
