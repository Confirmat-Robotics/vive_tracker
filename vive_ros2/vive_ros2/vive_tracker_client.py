"""Vive tracker UDP client for interacting with ROS 2 nodes."""

import argparse
import json
import logging
import socket
from os.path import expanduser
from pathlib import Path
from typing import Optional, Tuple

from vive_ros2.models import ViveDynamicObjectMessage


class ViveTrackerClient:
    """Continuously polls tracker data from a remote Vive tracker server."""

    def __init__(
        self,
        host: str,
        port: int,
        tracker_name: str,
        time_out: float = 1,
        buffer_length: int = 1024,
        should_record: bool = False,
        output_file_path: Path = Path(expanduser("~") + "/vive_ros2/data/RFS_Track.txt"),
    ) -> None:
        self.host = host
        self.port = port
        self.tracker_name: str = tracker_name
        self.time_out = time_out
        self.buffer_length = buffer_length
        self.socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20)
        self.socket.settimeout(self.time_out)
        self.latest_tracker_message: Optional[ViveDynamicObjectMessage] = None
        self.should_record = should_record
        self.output_file_path = output_file_path
        self.output_file = None
        if self.should_record:
            if self.output_file_path.parent.exists():
                self.output_file_path.parent.mkdir(exist_ok=True, parents=True)
            self.output_file = self.output_file_path.open("w")
        self.count = 0
        self.logger = logging.getLogger(f"Vive Tracker Client [{self.tracker_name}]")
        self.logger.info("Tracker Initialized")

    def update(self) -> None:
        """Poll the tracker and keep the latest parsed message in memory."""
        self.logger.info(
            f"Start Subscribing to [{self.host}:{self.port}] for [{self.tracker_name}] Vive Tracker Updates"
        )
        while True:
            try:
                _ = self.socket.sendto(self.tracker_name.encode(), (self.host, self.port))
                data, addr = self.socket.recvfrom(self.buffer_length)
                parsed_message, status = self.parse_message(data.decode())
                if status:
                    self.update_latest_tracker_message(parsed_message=parsed_message)
                    if self.should_record and self.count % 10 == 0:
                        self.output_file.write(
                            f"{self.latest_tracker_message.x},{self.latest_tracker_message.y},"
                            f"{self.latest_tracker_message.z},{self.latest_tracker_message.roll},"
                            f"{self.latest_tracker_message.pitch},{self.latest_tracker_message.yaw}\n"
                        )
                    self.count += 1
                else:
                    self.logger.error(f"Failed to parse incoming message [{data.decode()}]")
            except socket.timeout:
                self.logger.error("Timed out")
            except ConnectionResetError as exc:
                self.logger.error(f"Error: {exc}. Retrying")
            except OSError:
                pass
            except KeyboardInterrupt:
                raise SystemExit(1)
            except Exception as exc:  # pragma: no cover - best effort logging
                self.logger.debug(exc)

    def run_threaded(self, queue, kill) -> None:
        """Thread-friendly update loop that pushes parsed messages onto a queue."""
        self.logger.info(
            f"Start Subscribing to [{self.host}:{self.port}] for [{self.tracker_name}] Vive Tracker Updates"
        )
        while not kill.is_set():
            try:
                _ = self.socket.sendto(self.tracker_name.encode(), (self.host, self.port))
                data, addr = self.socket.recvfrom(self.buffer_length)
                parsed_message, status = self.parse_message(data.decode())
                if status:
                    self.update_latest_tracker_message(parsed_message=parsed_message)
                    queue.put(self.latest_tracker_message)
                    if self.should_record and self.count % 10 == 0:
                        self.output_file.write(
                            f"{self.latest_tracker_message.x},{self.latest_tracker_message.y},"
                            f"{self.latest_tracker_message.z},{self.latest_tracker_message.roll},"
                            f"{self.latest_tracker_message.pitch},{self.latest_tracker_message.yaw}\n"
                        )
                    self.count += 1
                else:
                    self.logger.error(f"Failed to parse incoming message [{data.decode()}]")
            except socket.timeout:
                self.logger.error("Timed out")
            except ConnectionResetError as exc:
                self.logger.error(f"Error: {exc}. Retrying")
            except OSError:
                pass
            except KeyboardInterrupt:
                raise SystemExit(1)
            except Exception as exc:  # pragma: no cover - best effort logging
                self.logger.debug(exc)

    def shutdown(self) -> None:
        """Safely close socket and stop recording."""
        self.socket.close()
        if self.output_file is not None:
            self.output_file.close()

    def update_latest_tracker_message(self, parsed_message: str) -> None:
        """Deserialize tracker JSON and store the newest message."""
        try:
            payload = json.loads(json.loads(parsed_message))
            vive_tracker_message = ViveDynamicObjectMessage.parse_obj(payload)
            if vive_tracker_message.device_name == self.tracker_name:
                self.latest_tracker_message = vive_tracker_message
            self.logger.debug(self.latest_tracker_message)
        except Exception as exc:  # pragma: no cover - best effort logging
            self.logger.error(
                f"Error: {exc} \nMaybe it is related to unable to parse buffer [{parsed_message}]. "
            )

    @staticmethod
    def parse_message(received_message: str) -> Tuple[str, bool]:
        """Parse incoming payload and ensure it matches expected framing."""
        start = received_message.find("&")
        end = received_message.find("\r")
        if start == -1 or end == -1:
            return "", False
        return received_message[start + 1 : end], True

    @staticmethod
    def initialize_socket() -> socket.socket:
        """Create a UDP socket configured for multicast use."""
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        soc.settimeout(3)
        soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        soc.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_TTL, 20)
        soc.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
        return soc


def str2bool(value):
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ("yes", "true", "t", "y", "1"):
        return True
    if lowered in ("no", "false", "f", "n", "0"):
        return False
    raise argparse.ArgumentTypeError("Boolean value expected.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", default=False, help="debug flag", type=str2bool)
    parser.add_argument("--collect", default=False, help="debug flag", type=str2bool)
    args = parser.parse_args()
    logging.basicConfig(
        format="%(asctime)s|%(name)s|%(levelname)s|%(message)s",
        datefmt="%H:%M:%S",
        level=logging.DEBUG if args.debug else logging.INFO,
    )
    host, port = "127.0.0.1", 8000
    client = ViveTrackerClient(host=host, port=port, tracker_name="tracker_1", should_record=args.collect)
    client.update()

if __name__ == "__main__":
    main()
