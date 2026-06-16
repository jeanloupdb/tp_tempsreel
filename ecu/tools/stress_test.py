#!/usr/bin/env python3
# script de stress test (repris du sujet) adapte pour parler a l'ecu sous qemu
# au lieu d'un /dev/ttyUSB reel. on se connecte au port serie expose par qemu
# via un socket tcp. mettre ECU_PORT=socket://localhost:5557 par exemple.
# en vrai hardware : ECU_PORT=/dev/ttyUSB0

import os
import struct
import time
import random
import threading

import serial

# --- config ---
SERIAL_PORT = os.environ.get('ECU_PORT', 'socket://localhost:5557')
BAUD_RATE = 115200
TIMEOUT = 0.1

MOTOR_GAIN = 1.65
DRAG_COEFF = 0.9
MODEL_RESPONSE = 1.2
MAX_SPEED = 220.0

# codes des messages
MSG_SETPOINT = 0x01
MSG_SPEED = 0x02
MSG_MODE_SET = 0x05
MSG_OUTPUT = 0x80
MSG_STATS = 0x83
MSG_ALARM = 0x85


class ECUTester:
    def __init__(self):
        # serial_for_url marche pour un device classique comme pour socket://
        self.ser = serial.serial_for_url(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
        self.current_speed = 50.0
        self.target_setpoint = 90.0
        self.last_output = 0.0
        self.running = True
        self.stats_count = 0
        self.start_time = time.time()

    def update_vehicle_model(self, dt):
        # modele 1er ordre : accel = moteur - trainee
        dt = max(0.0, min(dt, 0.2))
        acceleration = MODEL_RESPONSE * ((MOTOR_GAIN * self.last_output) -
                                         (DRAG_COEFF * self.current_speed))
        self.current_speed += acceleration * dt
        self.current_speed = max(0.0, min(MAX_SPEED, self.current_speed))

    def compute_crc(self, data_bytes):
        crc = 0
        for b in data_bytes:
            crc ^= b
        return crc

    def send_frame(self, msg_type, payload=b''):
        # [0xAA][LEN][TYPE][PAYLOAD][CRC]
        length = len(payload) + 1
        header = struct.pack('<HB', length, msg_type)
        full_msg = header + payload
        crc = self.compute_crc(full_msg)
        frame = struct.pack('B', 0xAA) + full_msg + struct.pack('B', crc)
        self.ser.write(frame)

    def receive_feedback(self):
        while self.running:
            if self.ser.in_waiting > 0:
                start_byte = self.ser.read(1)
                if start_byte == b'\xaa':
                    len_bytes = self.ser.read(2)
                    if len(len_bytes) < 2:
                        continue
                    length = struct.unpack('<H', len_bytes)[0]
                    data = self.ser.read(length)
                    crc_received = self.ser.read(1)
                    if len(data) == length and crc_received:
                        if self.compute_crc(len_bytes + data) == ord(crc_received):
                            msg_type = data[0]
                            payload = data[1:]
                            if msg_type == MSG_OUTPUT:
                                self.last_output = struct.unpack('<f', payload)[0]
                                print(f"[SPEED] ({self.current_speed:.2f} km/h) | commande: {self.last_output:.2f}")
                            elif msg_type == MSG_STATS:
                                self.stats_count += 1
                                print(f"[TELEMETRIE] recue ({self.stats_count}s)")
                            elif msg_type == MSG_ALARM:
                                print(f"\n[ALERTE ECU] : {payload.decode(errors='ignore')}")
            else:
                time.sleep(0.005)

    def run_normal_operation(self, duration):
        print(f"--- debut phase normale ({duration}s) ---")
        self.send_frame(MSG_SETPOINT, struct.pack('<f', self.target_setpoint))
        self.send_frame(MSG_MODE_SET, struct.pack('B', 2))   # mode auto

        end_time = time.time() + duration
        last_tick = time.time()
        while time.time() < end_time:
            now = time.time()
            dt = now - last_tick
            last_tick = now
            self.update_vehicle_model(dt)
            self.send_frame(MSG_SPEED, struct.pack('<f', self.current_speed))
            time.sleep(0.1)

    def run_stress_test(self):
        print("\n--- debut phase de stress ---")

        # 1. flood de messages (surcharge cpu)
        print("action: surcharge cpu (flood)...")
        for _ in range(50):
            self.send_frame(random.randint(0, 0xFF), os.urandom(4))

        # 2. trames fragmentees
        print("action: envoi de trame fragmentee...")
        partial_frame = struct.pack('B', 0xAA) + struct.pack('<HB', 5, MSG_SPEED)
        self.ser.write(partial_frame)
        time.sleep(0.5)   # pause au milieu du message
        self.ser.write(struct.pack('<f', 100.0) + b'\x00')

        # 3. erreur de crc
        print("action: envoi erreur crc...")
        self.ser.write(b'\xAA\x05\x00\x02\x00\x00\x00\x00\xFF')

    def test_failsafe(self):
        print("\n--- test failsafe (silence radio 2s) ---")
        time.sleep(2.5)
        if abs(self.last_output) < 1e-3:
            print("verification reussie : l'ecu a coupe la commande moteur (output=0)")
        else:
            print("echec : l'ecu n'est pas passe en mode failsafe")

    def start(self):
        reader = threading.Thread(target=self.receive_feedback, daemon=True)
        reader.start()
        try:
            self.run_normal_operation(30)
            self.run_stress_test()
            self.run_normal_operation(5)
            self.test_failsafe()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.ser.close()
            print("\ntest termine.")


if __name__ == "__main__":
    tester = ECUTester()
    tester.start()
