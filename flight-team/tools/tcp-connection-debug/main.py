import UAV_Computer_Vision.field_detection as fd
import cv2 as cv
import io
from PIL import Image
import numpy as np
from pathlib import Path
import socket
from enum import Enum
import logging
import sys

cv_module_path = Path("./UAV_Computer_Vision/")
sys.path.insert(1, cv_module_path)

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

output_image_file = Path("./output.jpg")


class MessageType(Enum):
    CAMERA_DATA = 1

    OK = 11
    ERROR = 12


class DebugClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = None
        self.handlers = {}

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        logging.info(f"Connected to {self.host}:{self.port}")

    def close(self):
        if not self.sock:
            return

        self.sock.close()
        logging.info(f"Connection to {self.host}:{self.port} closed")

    def register_handler(self, msg_type: MessageType, handler: callable):
        self.handlers[msg_type] = handler

    def send_request(self, msg_type: MessageType, payload: bytes = 0):
        self.sock.sendall(bytes(msg_type.value))
        self.sock.sendall(payload)

    def receive_n_bytes(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            buf = self.sock.recv(n - len(data))
            if len(buf) == 0:
                logging.warning("Connection closed unexpectedly")

            data += buf

        return data

    # just decide in the handlers what to do when not ok
    def wait_for_ok(self) -> int:
        response = self.receive_n_bytes(1)
        response = MessageType(response)

        if response == MessageType.OK:
            return 0

        logging.error("Error response from server!!!!")
        return -1

    def handle(self, msg_type: MessageType, payload: bytes = 0):
        handler = self.handlers.get(msg_type)
        handler(payload)  # man I love python


def downscale_image(image: cv.UMat) -> cv.UMat:
    img = Image.fromarray(image)
    new_width = img.size[0] // 3
    new_height = img.size[1] // 3
    img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
    return np.array(img)


def handle_camera(payload: bytes = 0):
    try:
        with open(output_image_file, "wb") as file:
            file.write(payload)
            logging.info(f"Camera output saved to {output_image_file}")
    except Exception as e:
        logging.error(f"Something bad happened :( {e}")


if __name__ == '__main__':
    client = DebugClient('localhost', 5000)

    client.register_handler(MessageType.CAMERA_DATA, handle_camera)

    client.connect()
    client.send_request(MessageType.CAMERA_DATA,
                        bytes(MessageType.CAMERA_DATA.value))

    size = int.from_bytes(client.receive_n_bytes(8))
    payload = client.receive_n_bytes(size)

    client.handle(MessageType.CAMERA_DATA, payload)

    client.close()
