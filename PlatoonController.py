import json
import time
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
        self.last_seen = {}

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
            data = json.loads(msg.payload.decode("utf-8"))
            robot_id = data.get("robot_id", "ukjent")
            self.robot_status[robot_id] = data
            self.last_seen[robot_id] = time.time()
            self.gui_update()
        except json.JSONDecodeError:
            print(f"Feil ved parsing av melding: {msg.payload}")
        except Exception as e:
            print(f"Feil ved behandling av melding: {e}")

    def gui_update(self):
        """Send oppdatering til GUI."""
        if hasattr(self, "gui_callback"):
            self.gui_callback()

    def remove_offline_robots(self, timeout=10):
        """Fjern roboter som ikke har sendt statusoppdateringer innen timeout."""
        current_time = time.time()
        offline_robots = [
            robot_id
            for robot_id, last_time in self.last_seen.items()
            if current_time - last_time > timeout
        ]
        for robot_id in offline_robots:
            self.robot_status.pop(robot_id, None)
            self.last_seen.pop(robot_id, None)
            print(f"Robot {robot_id} er fjernet fra listen (timeout).")

    def start(self):
        """Start MQTT-klienten og sjekk for offline roboter."""
        self.mqtt_client.connect(self.broker_ip, self.port, 60)

        def offline_checker():
            while True:
                self.remove_offline_robots()
                time.sleep(1)

        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
        threading.Thread(target=offline_checker, daemon=True).start()

    def send_command(self, command, target_robot=None):
        """Publiser en kommando til en spesifikk robot eller alle roboter."""
        if target_robot:
            topic = f"platoon/commands/{target_robot}"
        else:
            topic = "platoon/commands"
        self.mqtt_client.publish(topic, json.dumps(command))
        print(f"Kommando sendt til {target_robot or 'alle roboter'}: {command}")

    def update_status_display(self):
        """Oppdater status i GUI."""
        if hasattr(self, "status_text"):
            status_lines = []
            for robot_id, status in self.robot_status.items():
                front_robot = status.get("front_robot_id", "Ingen")
                role = status.get("role", "Ingen")
                speed = status.get("platoon_speed", "Ukjent")
                state = status.get("state", "Ukjent")
                queue_position = status.get("queue_position", "Ukjent")
                steering_value = status.get("control_value")
                status_lines.append(
                    f"{robot_id}({role}): Status={state}, Fart={speed}, Bak={front_robot}, Kø plassering={queue_position}, Styrings Vinkel={steering_value}"
                )
            self.status_text.set("\n".join(status_lines))

class PlatoonControllerGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Platoon Controller")
        self.controller.gui_callback = self.update_gui

        self.status_text = tk.StringVar()
        self.status_label = ttk.Label(
            self.root, text="Robot Status:", anchor="w", font=("Arial", 12, "bold")
        )
        self.status_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)

        self.status_box = ttk.Label(
            self.root,
            textvariable=self.status_text,
            relief="sunken",
            anchor="nw",
            wraplength=500,
            justify="left",
        )
        self.status_box.grid(row=1, column=0, columnspan=2, sticky="we", padx=10, pady=5)

        self.robot_list_label = ttk.Label(
            self.root, text="Velg robot:", anchor="w", font=("Arial", 10)
        )
        self.robot_list_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)

        self.robot_listbox = tk.Listbox(self.root, height=5)
        self.robot_listbox.grid(row=2, column=1, sticky="we", padx=10, pady=5)

        self.status_button = ttk.Button(
            self.root, text="Endre Status", command=self.change_status
        )
        self.status_button.grid(row=3, column=0, columnspan=2, pady=5)

        self.speed_label = ttk.Label(
            self.root, text="Ny Hastighet:", anchor="w", font=("Arial", 10)
        )
        self.speed_label.grid(row=4, column=0, sticky="w", padx=10, pady=5)

        self.speed_entry = ttk.Entry(self.root)
        self.speed_entry.grid(row=4, column=1, sticky="we", padx=10, pady=5)

        self.speed_button = ttk.Button(
            self.root, text="Oppdater Hastighet", command=self.update_speed
        )
        self.speed_button.grid(row=5, column=0, columnspan=2, pady=5)

        self.command_label = ttk.Label(
            self.root, text="Handling:", anchor="w", font=("Arial", 10)
        )
        self.command_label.grid(row=6, column=0, sticky="w", padx=10, pady=5)

        self.command_combobox = ttk.Combobox(
            self.root, values=["join_queue", "leave_queue", "change_leader"]
        )
        self.command_combobox.grid(row=6, column=1, sticky="we", padx=10, pady=5)

        self.command_button = ttk.Button(
            self.root, text="Utfør Handling", command=self.send_command
        )
        self.command_button.grid(row=7, column=0, columnspan=2, pady=5)

        controller.status_text = self.status_text
        threading.Thread(target=self.controller.start, daemon=True).start()
        self.root.mainloop()

    def update_gui(self):
        """Oppdater GUI-komponenter."""
        self.root.after(0, self._refresh_gui)

    def _refresh_gui(self):
        """Oppdater statusdisplay og robotliste."""
        try:
            self.update_status_display()
            self.update_robot_list()
        except Exception as e:
            print(f"Feil ved oppdatering av GUI: {e}")

    def update_status_display(self):
        """Oppdater status i GUI."""
        try:
            self.controller.update_status_display()
        except Exception as e:
            print(f"Feil ved oppdatering av statusdisplay: {e}")

    def update_robot_list(self):
        """Oppdater listen over tilgjengelige roboter i GUI."""
        try:
            selected_index = self.robot_listbox.curselection()
            selected_robot = self.robot_listbox.get(selected_index) if selected_index else None

            self.robot_listbox.delete(0, tk.END)
            robot_ids = list(self.controller.robot_status.keys())

            for robot_id in robot_ids:
                self.robot_listbox.insert(tk.END, robot_id)

            if selected_robot in robot_ids:
                new_index = robot_ids.index(selected_robot)
                self.robot_listbox.select_set(new_index)
                self.robot_listbox.activate(new_index)
            else:
                self.robot_listbox.selection_clear(0, tk.END)
                self.robot_listbox.activate(0)
        except Exception as e:
            print(f"Feil ved oppdatering av robotlisten: {e}")

    def update_speed(self):
        """Send ny hastighet som kommando."""
        try:
            speed = float(self.speed_entry.get())
            selected_robot = self.robot_listbox.get(tk.ACTIVE)
            if selected_robot:
                self.controller.send_command(
                    {"robot_id": selected_robot, "command": "set_speed", "speed": speed}, target_robot=selected_robot
                )
            else:
                print("Ingen robot valgt.")
        except ValueError:
            print("Ugyldig hastighet.")

    def change_status(self):
        """Send en kommando for å endre status på en valgt robot."""
        selected_robot = self.robot_listbox.get(tk.ACTIVE)
        if selected_robot:
            current_status = self.controller.robot_status.get(
                selected_robot, {}
            ).get("state", "UNKNOWN")
            new_status = "RUNNING" if current_status == "IDLE" else "IDLE"
            command = {"robot_id": selected_robot, "command": "change_status", "new_status": new_status}
            self.controller.send_command(command, target_robot=selected_robot)
        else:
            print("Ingen robot valgt.")

    def send_command(self):
        """Utfør en handling basert på valgt kommando."""
        selected_robot = self.robot_listbox.get(tk.ACTIVE)
        action = self.command_combobox.get()
        if selected_robot and action:
            command = {"robot_id": selected_robot, "command": action}
            self.controller.send_command(command, target_robot=selected_robot)
        else:
            print("Velg robot og handling.")


if __name__ == "__main__":
    broker_ip = "158.39.162.129"
    port = 1883

    controller = PlatoonController(broker_ip=broker_ip, port=port)

    PlatoonControllerGUI(controller)
