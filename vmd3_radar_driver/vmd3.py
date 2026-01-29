# coding: utf-8
"""
******************************************************************************
*  Copyright 2025 Labforge Inc.                                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License");            *
* you may not use this project except in compliance with the License.        *
* You may obtain a copy of the License at                                    *
*                                                                            *
*     https://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
******************************************************************************

Low-level driver for V-MD3 mmWave radar sensor.

"""
__author__ = "Thomas Reidemeister <thomas@labforge.ca>"
__copyright__ = "Copyright 2025, Labforge Inc."

import socket
import math
import enum
import time
import struct
from collections import namedtuple
import threading
import queue

# Try to get SO_TIMESTAMP from socket, else define manually
if hasattr(socket, "SO_TIMESTAMP"):
    SO_TIMESTAMP = socket.SO_TIMESTAMP
else:
    # https://www.kernel.org/doc/Documentation/networking/timestamping.txt
    SO_TIMESTAMP = 29  # Most common value on Linux

class VMD3Modes(enum.Enum):
    """
    Enumeration for V-MD3 operating modes.

    | # | Mode                    | Description                                                              |
    |---|-------------------------|--------------------------------------------------------------------------|
    | 0 | 2D, 6m, 10km/h 128/64   | 2D mode, 6m range, 10km/h speed, 128 raw returns, 64 tracked targets     |
    | 1 | 2D, 10m, 10km/h 128/64  | 2D mode, 10m range, 10km/h speed, 128 raw returns, 64 tracked targets    |
    | 2 | 2D, 30m, 30km/h 128/64  | 2D mode, 30m range, 30km/h speed, 128 raw returns, 64 tracked targets    |
    | 3 | 2D, 30m, 50km/h 128/64  | 2D mode, 30m range, 50km/h speed, 128 raw returns, 64 tracked targets    |
    | 4 | 2D, 50m, 50km/h 128/64  | 2D mode, 50m range, 50km/h speed, 128 raw returns, 64 tracked targets    |
    | 5 | 2D, 100m, 100km/h 128/64| 2D mode, 100m range, 1000km/h speed, 128 raw returns, 64 tracked targets |
    | 6 | 3D, 6m, 10km/h 128/64   | 3D mode, 6m range, 10km/h speed, 128 raw returns, 64 tracked targets     |
    | 7 | 3D, 10m, 10km/h 128/64  | 3D mode, 10m range, 10km/h speed, 128 raw returns, 64 tracked targets    |
    | 8 | 3D, 30m, 30km/h 128/64  | 3D mode, 30m range, 30km/h speed, 128 raw returns, 64 tracked targets    |

    """
    MODE_2D_6M_10KMH_128_64 = 0
    MODE_2D_10M_10KMH_128_64 = 1
    MODE_2D_30M_30KMH_128_64 = 2
    MODE_2D_30M_50KMH_128_64 = 3
    MODE_2D_50M_50KMH_128_64 = 4
    MODE_2D_100M_100KMH_128_64 = 5
    MODE_3D_6M_10KMH_128_64 = 6
    MODE_3D_10M_10KMH_128_64 = 7
    MODE_3D_30M_30KMH_128_64 = 8

# 0 = OK
# 1–3 = Reserved
# 4 = Bootloader entry
# 5 = Invalid HEX checksum
# 6 = Invalid HEX record
# 7 = Not enough memory
# 8 = Flash error
class VMD3Errors(enum.Enum):
    """
    Enumeration for V-MD3 error codes.
    """
    OK = 0
    RESERVED_1 = 1
    RESERVED_2 = 2
    RESERVED_3 = 3
    BOOTLOADER_ENTRY = 4
    INVALID_HEX_CHECKSUM = 5
    INVALID_HEX_RECORD = 6
    NOT_ENOUGH_MEMORY = 7
    FLASH_ERROR = 8


RadarDetection = namedtuple('RadarDetection', ['distance_cm', 'speed_kmh', 'azimuth_rad', 'elevation_rad', 'magnitude'])

class VMD3Driver:
    """
    Low-level driver for V-MD3 mmWave radar sensor.
    """

    @staticmethod
    def recv_msg(sock_tcp, msg_len, max_msg_size):
        """
        Helper for receiving messages over TCP/IP.
        :param sock_tcp: TCP socket
        :param msg_len: Total length of the message to receive
        :param max_msg_size: Maximum size of each read operation
        :return: Received message as bytes
        """
        resp_frame = bytearray(msg_len)
        pos = 0
        while pos < msg_len:
            resp_frame[pos:pos + max_msg_size] = sock_tcp.recv(max_msg_size)
            pos += max_msg_size
        return resp_frame

    @staticmethod
    def command(cmd: str, payload: bytes) -> bytes:
        """
        Helper for creating command frames to be sent over TCP/IP.
        :param cmd: Command string (4 characters)
        :param payload: Payload as bytes
        :return: Complete command frame as bytes
        """
        header = bytes(cmd, 'utf-8')
        payload_length = len(payload).to_bytes(4, byteorder='little')
        cmd_frame = header + payload_length + payload
        return cmd_frame

    def __init__(self, tcp_ip: str = '192.168.100.201', tcp_port: int = 6172, udp_port: int = 4567):
        """
        Initialize the V-MD3 driver by establishing TCP and UDP connections.
        :param tcp_ip: IP address of the V-MD3 sensor for TCP connection.
        :param tcp_port: TCP port number for the V-MD3 sensor.
        :param udp_port: UDP port number for receiving data from the V-MD3 sensor
        """
        self._sock_udp = None
        self._sock_tcp = None
        # Create TCP/IP socket
        self._sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock_tcp.settimeout(10)  # 10 second timeout
        self._sock_tcp.connect((tcp_ip, tcp_port))

        self._sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        local_ip = self._sock_tcp.getsockname()[0]
        self._sock_udp.bind((local_ip, udp_port))
        self._sock_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_udp.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMP, 1)

        # Initialize connection with sensor
        cmd_frame = self.command("INIT", b'')
        self._sock_tcp.send(cmd_frame)

        self._udp_queue = queue.Queue(maxsize=100)
        self._receiver_running = True
        self._receiver_thread = threading.Thread(target=self._udp_receiver, daemon=True)
        self._receiver_thread.start()

    def _udp_receiver(self):
        packet_length = 1500
        while self._receiver_running:
            try:
                #packet, _ = self._sock_udp.recvfrom(packet_length)
                packet, ancdata, flags, addr = self._sock_udp.recvmsg(packet_length, 1024)
                packet_timestamp = time.time() # Fallback timestamp
                for cmsg_level, cmsg_type, cmsg_data in ancdata:
                    if cmsg_level == socket.SOL_SOCKET and cmsg_type == SO_TIMESTAMP:
                        tv_sec, tv_usec = struct.unpack('ll', cmsg_data)
                        packet_timestamp = tv_sec + tv_usec / 1e6 # Packet receive timestamp

                self._udp_queue.put((packet, packet_timestamp))
            except Exception:
                import traceback
                traceback.print_exc()
                continue

    def check_send(self, command: bytes):
        """
        Send a command and check for acknowledgment.
        :param command: Command frame as bytes.
        """
        self._sock_tcp.send(command)
        resp_frame = self.recv_msg(self._sock_tcp, 9, 8)
        # Assuming RESP packet structure: [CMD(4 bytes), LEN(4 bytes), STATUS(1 byte)]
        res = VMD3Errors(resp_frame[8])
        if res != VMD3Errors.OK:
            raise IOError(f'Error: Command not acknowledged code = {res.name}')

    def mode(self, mode: VMD3Modes):
        """
        Set the operating mode of the V-MD3 sensor.
        :param mode: VMD3Modes enum value representing the desired mode.
        """
        # Set operating mode
        mode_payload = mode.value.to_bytes(4, byteorder='little')
        cmd_frame = self.command("RSET", mode_payload)
        self.check_send(cmd_frame)

    def enable_static_tracking(self, enable: bool):
        """
        Enable or disable static target tracking.
        :param enable: True to enable static tracking, False to disable.
        """
        # Enable/disable static target tracking
        enable_payload = (1 if enable else 0).to_bytes(4, byteorder='little')
        cmd_frame = self.command("STOB", enable_payload)
        self.check_send(cmd_frame)

    def sensitivity(self, level: int):
        """
        Set the sensitivity level of the V-MD3 sensor.
        :param level: Sensitivity level (0-15).
        """
        if level < 0 or level > 15:
            raise ValueError("Sensitivity level must be between 0 and 15.")
        # Set sensitivity level
        sensitivity_payload = level.to_bytes(4, byteorder='little')
        cmd_frame = self.command("SENS", sensitivity_payload)
        self.check_send(cmd_frame)

    def minimum_detection_distance(self, distance_pct: int):
        """
        Set the minimum detection distance.
        :param distance_pct: Minimum detection distance in percent of detection range.
        """
        distance_payload = distance_pct.to_bytes(4, byteorder='little')
        cmd_frame = self.command("MIRA", distance_payload)
        self.check_send(cmd_frame)

    def maximum_detection_distance(self, distance_pct: int):
        """
        Set the maximum detection distance.
        :param distance_pct: Maximum detection distance in percent of detection range.
        """
        distance_payload = distance_pct.to_bytes(4, byteorder='little')
        cmd_frame = self.command("MARA", distance_payload)
        self.check_send(cmd_frame)

    def minimum_detection_speed(self, speed_pct: int):
        """
        Set the minimum detection speed.
        :param speed_pct: Minimum detection speed in percent of maximum radar mode speed.
        """
        speed_payload = speed_pct.to_bytes(4, byteorder='little')
        cmd_frame = self.command("MISP", speed_payload)
        self.check_send(cmd_frame)

    def maximum_detection_speed(self, speed_pct: int):
        """
        Set the maximum detection speed.
        :param speed_pct: Maximum detection speed in percent of maximum radar mode speed.
        """
        speed_payload = speed_pct.to_bytes(4, byteorder='little')
        cmd_frame = self.command("MASP", speed_payload)
        self.check_send(cmd_frame)

    def minimum_lifetime(self, lifetime_frames: int):
        """
        Set the minimum lifetime for tracked targets.
        :param lifetime_frames: Minimum lifetime in frames (0-100).
        """
        if lifetime_frames < 0 or lifetime_frames > 100:
            raise ValueError("Lifetime frames must be between 0 and 100.")
        lifetime_payload = lifetime_frames.to_bytes(4, byteorder='little')
        cmd_frame = self.command("TVLT", lifetime_payload)
        self.check_send(cmd_frame)

    def maximum_lifetime(self, lifetime_frames: int):
        """
        Set the maximum lifetime for tracked targets.
        :param lifetime_frames: Maximum lifetime in frames (0-100).
        """
        if lifetime_frames < 0 or lifetime_frames > 100:
            raise ValueError("Lifetime frames must be between 0 and 100.")
        lifetime_payload = lifetime_frames.to_bytes(4, byteorder='little')
        cmd_frame = self.command("TDLT", lifetime_payload)
        self.check_send(cmd_frame)

    def stream_enable(self, pdat=True, tdat=True):
        """
        Enable streaming of PDAT and TDAT data.
        """
        # Enable PDAT and TDAT data
        # 0x10 = TDAT, 0x08 = PDAT, 0x20 = DONE
        value = 0x20
        if pdat:
            value |= 0x08
        if tdat:
            value |= 0x10
        cmd_frame = self.command("RDOT", value.to_bytes(4, byteorder='little'))
        self._sock_tcp.send(cmd_frame)
        resp_frame = self.recv_msg(self._sock_tcp, 9, 8)
        res = VMD3Errors(resp_frame[8])
        if res != VMD3Errors.OK:
            raise IOError(f'Error: Command not acknowledged code = {res.name}')

    @staticmethod
    def decode_packet(packet):
        """
        Decode a received UDP packet from the V-MD3 sensor.
        :param packet: Received packet as bytes.
        :return: Tuple containing packet type and list of RadarDetection objects, or frame count for 'DONE' packets.
        """
        if packet[0:4] == b'PDAT' or packet[0:4] == b'TDAT':
            data = []
            packet_type = packet[0:4].decode('ascii')
            resp_len = int.from_bytes(packet[4:8], byteorder='little')  # get response length
            num_targets = round(resp_len / 10)  # calculate number of detected targets
            packet = packet[8:len(packet)]  # exclude header from data
            for target in range(0, num_targets):
                distance_cm = int.from_bytes(packet[10 * target:10 * target + 2], byteorder='little', signed=False)
                speed_kph = (
                    int.from_bytes(packet[10 * target + 2:10 * target + 4], byteorder='little', signed=True) / 100)
                azimuth = (
                  math.radians(int.from_bytes(packet[10 * target + 4:10 * target + 6], byteorder='little', signed=True)
                               / 100))
                elevation = (
                  math.radians(int.from_bytes(packet[10 * target + 6:10 * target + 8], byteorder='little', signed=True)
                               / 100))
                magnitude = int.from_bytes(packet[10 * target + 8:10 * target + 10], byteorder='little', signed=False)
                # exclude bad detections
                if distance_cm == 0 and speed_kph == 0 and azimuth == 0 and elevation == 0 and magnitude == 0:
                    continue
                data.append(RadarDetection(distance_cm, speed_kph, azimuth, elevation, magnitude))
            return packet_type, data
        if packet[0:4] == b'DONE':
            frame_count = int.from_bytes(packet[8:12], byteorder='little', signed=False)
            return 'DONE', frame_count

        return None

    def __next__(self):
        # Pull from the queue, blocking if empty
        while True:
            packet, packet_timestamp = self._udp_queue.get()
            result = self.decode_packet(packet)
            if result is not None:
                return (*result, packet_timestamp)

    def __del__(self):
        self._receiver_running = False
        self._receiver_thread.join(timeout=1)

        # Disconnect from sensor
        cmd_frame = self.command("GBYE", b'')
        try:
            self._sock_tcp.send(cmd_frame)
        except: # pylint: disable=bare-except
            pass

        # Close connections
        if self._sock_udp is not None:
            self._sock_udp.close()
        if self._sock_udp is not None:
            self._sock_tcp.close()

if __name__ == '__main__':
    driver = VMD3Driver()
    driver.mode(VMD3Modes.MODE_3D_6M_10KMH_128_64)
    driver.sensitivity(14)
    driver.stream_enable()

    try:
        while True:
            (data_type, received_data) = next(driver)
            if data_type in ['PDAT', 'TDAT']:
                print(f"Received {len(received_data)} detections of type {data_type}:")
                for det in received_data:
                    print(f"  Distance: {det.distance_cm} cm, Speed: {det.speed_kmh} km/h, "
                          f"Azimuth: {math.degrees(det.azimuth_rad):.2f} deg, "
                          f"Elevation: {math.degrees(det.elevation_rad):.2f} deg, Magnitude: {det.magnitude}")
            elif data_type == 'DONE':
                print(f"Frame complete. Total frames processed: {received_data}")

    except KeyboardInterrupt:
        print("Exiting...")
