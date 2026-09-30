"""Small MQTT 3.1.1 client for MicroPython ESP8266 firmware."""

import socket
import struct


class MQTTException(Exception):
    pass


class MQTTClient:
    def __init__(
        self,
        client_id,
        server,
        port=1883,
        user=None,
        password=None,
        keepalive=60,
    ):
        self.client_id = client_id
        self.server = server
        self.port = port
        self.user = user
        self.password = password
        self.keepalive = keepalive
        self.sock = None
        self.cb = None

    @staticmethod
    def _encode_string(value):
        encoded = value.encode()
        return struct.pack("!H", len(encoded)) + encoded

    @staticmethod
    def _encode_remaining_length(length):
        output = bytearray()
        while True:
            digit = length % 128
            length //= 128
            if length:
                digit |= 0x80
            output.append(digit)
            if not length:
                return output

    def _wait_msg(self):
        first_byte = self.sock.read(1)
        if not first_byte:
            return None

        byte1 = first_byte[0]
        msg_type = byte1 >> 4
        msg_qos = (byte1 >> 1) & 0x03
        remaining_length = 0
        multiplier = 1

        while True:
            digit = self.sock.read(1)
            if not digit:
                raise MQTTException("connection closed")
            digit = digit[0]
            remaining_length += (digit & 127) * multiplier
            if not (digit & 128):
                break
            multiplier *= 128
            if multiplier > 128 * 128 * 128:
                raise MQTTException("malformed remaining length")

        payload = self.sock.read(remaining_length) if remaining_length else b""
        return msg_type, msg_qos, payload

    def connect(self):
        address = socket.getaddrinfo(self.server, self.port)[0][-1]
        self.sock = socket.socket()
        self.sock.settimeout(5)
        self.sock.connect(address)

        flags = 0x02
        if self.user is not None:
            flags |= 0x80
            if self.password is not None:
                flags |= 0x40

        variable_header = bytearray(b"\x00\x04MQTT\x04")
        variable_header.append(flags)
        variable_header.extend(struct.pack("!H", self.keepalive))

        payload = bytearray(self._encode_string(self.client_id))
        if self.user is not None:
            payload.extend(self._encode_string(self.user))
            if self.password is not None:
                payload.extend(self._encode_string(self.password))

        packet = variable_header + payload
        self.sock.write(b"\x10")
        self.sock.write(self._encode_remaining_length(len(packet)))
        self.sock.write(packet)

        response = self._wait_msg()
        if response is None or response[0] != 2 or len(response[2]) < 2:
            raise MQTTException("invalid CONNACK")
        if response[2][1] != 0:
            raise MQTTException("MQTT connection refused")
        return response[2][1]

    def set_callback(self, callback):
        self.cb = callback

    def subscribe(self, topic, qos=0):
        packet_id = 1
        encoded_topic = topic.encode()
        payload = bytearray(struct.pack("!H", packet_id))
        payload.extend(struct.pack("!H", len(encoded_topic)))
        payload.extend(encoded_topic)
        payload.append(qos)

        self.sock.write(b"\x82")
        self.sock.write(self._encode_remaining_length(len(payload)))
        self.sock.write(payload)

    def check_msg(self):
        if self.sock is None:
            return

        self.sock.settimeout(0.01)
        try:
            message = self._wait_msg()
            if message is None:
                return
            msg_type, _msg_qos, payload = message
            if msg_type != 3 or len(payload) < 2:
                return

            topic_length = (payload[0] << 8) | payload[1]
            topic_end = 2 + topic_length
            topic = payload[2:topic_end].decode()
            if self.cb:
                self.cb(topic, payload[topic_end:])
        except OSError:
            pass
        finally:
            self.sock.settimeout(5)

    def disconnect(self):
        if self.sock is None:
            return
        try:
            self.sock.write(b"\xe0\x00")
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None
