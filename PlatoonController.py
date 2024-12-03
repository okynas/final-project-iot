import json
import threading
import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt

class PlatoonController:
    def __init__(self, broker_ip, port):
        self.broker_ip = broker_ip
        self.port = port

        self.mqtt_client = mqtt.Client(client_id="PlatoonController", protocol=mqtt.MQTTv311, transport="tcp")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.robot_status = {}

    def on_connect(self, client, userdata, flags, rc):
        """Callback når vi kobler til MQTT-broker."""
        if rc == 0:
            print("Koblet til MQTT Broker")
            self.mqtt_client.subscribe("platoon/status/#")
            print("Abonnert på 'platoon/status'")
        else:
            print(f"Feil ved tilkobling til broker: {rc}")

    def on_message(self, client, userdata, msg):
        """Callback for å motta meldinger fra roboter."""
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            robot_id = data.get("robot_id", "ukjent")
            self.robot_status[robot_id] = data
            print(f"Oppdatert status for {robot_id}: {self.robot_status[robot_id]}")
            self.update_status_display()
        except json.JSONDecodeError:
            print(f"Feil ved parsing av melding: {msg.payload}")
        except Exception as e:
            print(f"Feil ved behandling av melding: {e}")

    def start(self):
        """Start MQTT-klienten."""
        self.mqtt_client.connect(self.broker_ip, self.port, 60)
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

    def send_command(self, command):
        """Publiser en kommando til robotene."""
        self.mqtt_client.publish("platoon/commands", json.dumps(command))
        print(f"Kommando sendt: {command}")

    def update_status_display(self):
        """Oppdater status i GUI."""
        if hasattr(self, 'status_text'):
            status_lines = []
            for robot_id, status in self.robot_status.items():
                leader = " (LEDER)" if status.get("is_leader", False) else ""
                front_robot = status.get("front_robot_id", "Ingen")
                speed = status.get("platoon_speed", "Ukjent")
                state = status.get("state", "Ukjent")
                status_lines.append(
                    f"{robot_id}{leader}: Status={state}, Fart={speed}, Bak={front_robot}"
                )
            self.status_text.set("\n".join(status_lines))

class PlatoonControllerGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Platoon Controller")

        self.status_text = tk.StringVar()
        self.status_label = ttk.Label(self.root, text="Robot Status:", anchor="w", font=("Arial", 12, "bold"))
        self.status_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)

        self.status_box = ttk.Label(self.root, textvariable=self.status_text, relief="sunken", anchor="nw", wraplength=500, justify="left")
        self.status_box.grid(row=1, column=0, columnspan=2, sticky="we", padx=10, pady=5)

        self.robot_list_label = ttk.Label(self.root, text="Velg robot:", anchor="w", font=("Arial", 10))
        self.robot_list_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)

        self.robot_listbox = tk.Listbox(self.root, height=5)
        self.robot_listbox.grid(row=2, column=1, sticky="we", padx=10, pady=5)

        self.status_button = ttk.Button(self.root, text="Endre Status", command=self.change_status)
        self.status_button.grid(row=3, column=0, columnspan=2, pady=5)

        self.speed_label = ttk.Label(self.root, text="Ny Hastighet:", anchor="w", font=("Arial", 10))
        self.speed_label.grid(row=4, column=0, sticky="w", padx=10, pady=5)

        self.speed_entry = ttk.Entry(self.root)
        self.speed_entry.grid(row=4, column=1, sticky="we", padx=10, pady=5)

        self.speed_button = ttk.Button(self.root, text="Oppdater Hastighet", command=self.update_speed)
        self.speed_button.grid(row=5, column=0, columnspan=2, pady=5)

        controller.status_text = self.status_text
        self.update_status_display()

        threading.Thread(target=self.controller.start, daemon=True).start()
        self.root.mainloop()

    def update_status_display(self):
        """Oppdater GUI for status."""
        self.controller.update_status_display()

        self.robot_listbox.delete(0, tk.END)
        for robot_id in self.controller.robot_status.keys():
            print(f"Legger til {robot_id} i robotlisten.")
            self.robot_listbox.insert(tk.END, robot_id)

    def update_speed(self):
        """Send ny hastighet som kommando."""
        try:
            speed = float(self.speed_entry.get())
            self.controller.send_command({"platoon_speed": speed})
        except ValueError:
            print("Ugyldig hastighet")

    def change_status(self):
        """Send en kommando for å endre status på en valgt robot."""
        selected_robot = self.robot_listbox.get(tk.ACTIVE)
        if selected_robot:
            current_status = self.controller.robot_status.get(selected_robot, {}).get("state", "UNKNOWN")
            new_status = "RUNNING" if current_status == "IDLE" else "IDLE"
            command = {"robot_id": selected_robot, "command": "change_status", "new_status": new_status}
            self.controller.send_command(command)
            print(f"Endret status for {selected_robot} til {new_status}")

if __name__ == "__main__":
    broker_ip = "158.39.162.129"
    port = 1883

    controller = PlatoonController(broker_ip=broker_ip, port=port)

    PlatoonControllerGUI(controller)
