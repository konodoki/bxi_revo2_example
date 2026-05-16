#!/usr/bin/env python3
"""BxiPci ROS CAN/CANFD bridge for the Python Stark SDK examples."""

from __future__ import annotations

import atexit
import inspect
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Deque, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from communication.msg import CANFDPacket
except ImportError as exc:  # pragma: no cover - depends on sourced ROS workspace
    CANFDPacket = None
    _communication_import_error = exc
else:
    _communication_import_error = None

try:
    from bc_stark_sdk import main_mod as libstark
except ImportError as exc:  # pragma: no cover - depends on installed SDK wheel
    libstark = None
    _sdk_import_error = exc
else:
    _sdk_import_error = None

if TYPE_CHECKING:
    from bc_stark_sdk import main_mod as stark_sdk

    StarkDeviceContext = stark_sdk.DeviceContext
else:
    StarkDeviceContext = Any if libstark is None else libstark.DeviceContext


BXI_PCI_DEFAULT_BUS_INDEX = 5
BXI_PCI_RX_WAIT_TIME_MS = 250
BXI_PCI_RX_BUFFER_SIZE = 1000

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CANFD_BRS = 0x01
CANFD_FDF = 0x04
MAX_CAN_DATA_LEN = 8
MAX_CANFD_DATA_LEN = 64


@dataclass
class RxFrame:
    bus: int
    can_id: int
    data: bytes


class BxiRosCanBridge(Node):
    def __init__(self, max_rx_frames: int = BXI_PCI_RX_BUFFER_SIZE) -> None:
        if CANFDPacket is None:
            raise RuntimeError(
                "communication.msg.CANFDPacket not found. "
                "Please source the ROS workspace that provides the communication package."
            ) from _communication_import_error

        super().__init__("bxi_revo2_can_node_py")
        self._max_rx_frames = max_rx_frames
        self._rx_frames: Deque[RxFrame] = deque()
        self._rx_cv = threading.Condition()
        self._bus_lock = threading.Lock()
        self._default_bus = BXI_PCI_DEFAULT_BUS_INDEX
        self._default_is_canfd = True
        self._slave_bus: Dict[int, int] = {}
        self._slave_is_canfd: Dict[int, bool] = {}

        qos = QoSProfile(depth=100)
        self._rx_sub = self.create_subscription(
            CANFDPacket, "canfd_packet/rx", self._on_rx_packet, qos
        )
        self._tx_pub = self.create_publisher(CANFDPacket, "canfd_packet/tx", qos)

    def set_default_bus(self, bus: int) -> None:
        with self._bus_lock:
            self._default_bus = int(bus)

    def set_default_protocol(self, is_canfd: bool) -> None:
        with self._bus_lock:
            self._default_is_canfd = bool(is_canfd)

    def register_slave_bus(self, slave_id: int, bus: int) -> None:
        with self._bus_lock:
            self._slave_bus[int(slave_id) & 0xFF] = int(bus)

    def register_slave(self, slave_id: int, bus: int, is_canfd: bool) -> None:
        with self._bus_lock:
            key = int(slave_id) & 0xFF
            self._slave_bus[key] = int(bus)
            self._slave_is_canfd[key] = bool(is_canfd)

    def bus_for_slave(self, slave_id: int) -> int:
        with self._bus_lock:
            return self._slave_bus.get(int(slave_id) & 0xFF, self._default_bus)

    def is_canfd_for_slave(self, slave_id: int) -> bool:
        with self._bus_lock:
            return self._slave_is_canfd.get(
                int(slave_id) & 0xFF, self._default_is_canfd
            )

    def send_packet(
        self,
        bus: int,
        can_id: int,
        data: bytes,
        flags: int = 0,
        max_data_len: int = MAX_CANFD_DATA_LEN,
    ) -> bool:
        payload = bytes(data[:max_data_len])

        packet = CANFDPacket()
        packet.bus = int(bus)
        packet.frame.can_id = int(can_id)
        packet.frame.flags = int(flags)
        packet.frame.len = len(payload)
        self._set_frame_data(packet, payload)
        self._tx_pub.publish(packet)
        frame_type = "CANFD" if (packet.frame.flags & CANFD_FDF) else "CAN"
        self.get_logger().debug(
            f"{frame_type} TX bus={packet.bus} id=0x{packet.frame.can_id:08X} "
            f"len={packet.frame.len} flags=0x{packet.frame.flags:02X}"
        )
        return True

    def receive_can(
        self,
        expected_bus: int,
        expected_can_id: int,
        expected_frames: int,
        wait_time_ms: int,
    ) -> Optional[Tuple[int, bytes]]:
        expected_can_id = int(expected_can_id) & CAN_EFF_MASK
        is_dfu_mode = expected_can_id == 0
        cmd = (expected_can_id >> 3) & 0x0F
        is_multi_frame_cmd = cmd in (0x0B, 0x0D)
        if is_dfu_mode:
            max_attempts = 200
        elif expected_frames > 1 or is_multi_frame_cmd:
            max_attempts = 5
        else:
            max_attempts = 2

        def matches(frame: RxFrame) -> bool:
            if frame.bus != expected_bus:
                return False
            return is_dfu_mode or expected_can_id == 0 or frame.can_id == expected_can_id

        all_data = bytearray()
        received_count = 0

        for _ in range(max_attempts):
            frame = self._wait_for_frame(matches, wait_time_ms / 1000.0)
            if frame is None:
                continue

            can_id = frame.can_id
            frame_data = frame.data

            if is_multi_frame_cmd and frame_data:
                frame_header = frame_data[0]

                if cmd == 0x0B and len(frame_data) >= 2:
                    len_and_flag = frame_data[1]
                    is_last = (len_and_flag & 0x80) != 0
                    all_data.extend(frame_data)
                    received_count += 1

                    if is_last:
                        return can_id, bytes(all_data)
                    continue

                if cmd == 0x0D:
                    total = (frame_header >> 4) & 0x0F
                    seq = frame_header & 0x0F

                    if total > 0 and seq > 0:
                        all_data.extend(frame_data)
                        received_count += 1

                        if received_count >= total:
                            return can_id, bytes(all_data)
                        continue

            all_data.extend(frame_data)
            return can_id, bytes(all_data)

        if all_data:
            return expected_can_id, bytes(all_data)

        return None

    def receive_canfd(
        self, expected_bus: int, expected_can_id: int, wait_time_ms: int
    ) -> Optional[Tuple[int, bytes]]:
        expected_slave_id = (int(expected_can_id) >> 16) & 0xFF
        expected_master_id = (int(expected_can_id) >> 8) & 0xFF

        def matches(frame: RxFrame) -> bool:
            if frame.bus != expected_bus:
                return False
            resp_slave_id = (frame.can_id >> 16) & 0xFF
            resp_master_id = (frame.can_id >> 8) & 0xFF
            return resp_slave_id == expected_slave_id and resp_master_id == expected_master_id

        for _ in range(2):
            frame = self._wait_for_frame(matches, wait_time_ms / 1000.0)
            if frame is not None:
                return frame.can_id, frame.data
        return None

    def clear_rx_queue(self) -> None:
        with self._rx_cv:
            self._rx_frames.clear()
            self._rx_cv.notify_all()

    def _on_rx_packet(self, msg: Any) -> None:
        frame_len = min(int(msg.frame.len), MAX_CANFD_DATA_LEN)
        frame = RxFrame(
            bus=int(msg.bus),
            can_id=int(msg.frame.can_id) & CAN_EFF_MASK,
            data=bytes(msg.frame.data[:frame_len]),
        )

        with self._rx_cv:
            if len(self._rx_frames) >= self._max_rx_frames:
                self._rx_frames.popleft()
            self._rx_frames.append(frame)
            self._rx_cv.notify_all()

        self.get_logger().debug(
            f"CANFD RX bus={frame.bus} id=0x{frame.can_id:08X} len={len(frame.data)}"
        )

    def _wait_for_frame(
        self, matches: Callable[[RxFrame], bool], timeout_sec: float
    ) -> Optional[RxFrame]:
        deadline = time.monotonic() + timeout_sec

        with self._rx_cv:
            while True:
                for index, frame in enumerate(self._rx_frames):
                    if matches(frame):
                        del self._rx_frames[index]
                        return frame

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None

                notified = self._rx_cv.wait(remaining)
                if not notified:
                    return None

    @staticmethod
    def _set_frame_data(packet: Any, payload: bytes) -> None:
        values = list(payload)
        try:
            packet.frame.data = values
            return
        except Exception:
            pass

        padded = values + [0] * (MAX_CANFD_DATA_LEN - len(values))
        try:
            packet.frame.data = padded
            return
        except Exception:
            pass

        for index, value in enumerate(values):
            packet.frame.data[index] = value


class BxiPciRosRuntime:
    _instance: Optional["BxiPciRosRuntime"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def instance(cls) -> "BxiPciRosRuntime":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: Optional[BxiRosCanBridge] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._owns_rclpy = False
        self._device_count = 0
        atexit.register(self.stop)

    def ensure_bridge(self, default_bus: int) -> BxiRosCanBridge:
        with self._lock:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True

            if self._node is None:
                self._node = BxiRosCanBridge()
                self._executor = MultiThreadedExecutor(num_threads=2)
                self._executor.add_node(self._node)
                self._thread = threading.Thread(
                    target=self._spin, name="bxi-pci-ros-runtime", daemon=True
                )
                self._thread.start()
                self._node.get_logger().info("BxiPci ROS CAN bridge initialized")

            self._node.set_default_bus(default_bus)
            return self._node

    def current_bridge(self) -> Optional[BxiRosCanBridge]:
        with self._lock:
            return self._node

    def register_device(self) -> None:
        with self._lock:
            self._device_count += 1

    def release_device(self) -> None:
        should_stop = False
        with self._lock:
            if self._device_count > 0:
                self._device_count -= 1
            should_stop = self._device_count == 0
        if should_stop:
            self.stop()

    def stop(self) -> None:
        with self._lock:
            executor = self._executor
            node = self._node
            thread = self._thread
            owns_rclpy = self._owns_rclpy
            self._executor = None
            self._node = None
            self._thread = None
            self._owns_rclpy = False

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass

        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass

        if node is not None:
            node.clear_rx_queue()
            node.destroy_node()

        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    def _spin(self) -> None:
        try:
            executor = self._executor
            if executor is not None:
                executor.spin()
        except ExternalShutdownException:
            pass


@dataclass
class BxiDeviceContext:
    handle: StarkDeviceContext
    bus: int
    slave_id: int
    is_canfd: bool = True
    protocol_type: Any = None
    hw_type: Any = None
    _closed: bool = False

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True

        try:
            close_device_handler = getattr(libstark, "close_device_handler", None)
            if close_device_handler is not None and self.handle is not None:
                result = close_device_handler(self.handle)
                if inspect.isawaitable(result):
                    await result
        finally:
            BxiPciRosRuntime.instance().release_device()
            self.handle = None


def _check_sdk() -> None:
    if libstark is None:
        raise RuntimeError(
            "bc_stark_sdk not found. Install the Python SDK wheel before running this example."
        ) from _sdk_import_error


_callback_lock = threading.Lock()
_callbacks_installed = False
_can_tx_callback: Optional[Callable[[int, int, Any], bool]] = None
_can_rx_callback: Optional[Callable[[int, int, int], Tuple[int, bytes]]] = None


def setup_bxipci_callbacks() -> None:
    _check_sdk()
    runtime = BxiPciRosRuntime.instance()
    global _callbacks_installed, _can_tx_callback, _can_rx_callback

    with _callback_lock:
        if _callbacks_installed:
            return

        def can_send(slave_id: int, can_id: int, data: Any) -> bool:
            bridge = runtime.current_bridge()
            if bridge is None:
                return False

            bus = bridge.bus_for_slave(slave_id)
            is_canfd = bridge.is_canfd_for_slave(slave_id)
            if is_canfd:
                packet_id = (int(can_id) & CAN_EFF_MASK) | CAN_EFF_FLAG
                return bridge.send_packet(
                    bus,
                    packet_id,
                    bytes(data),
                    CANFD_BRS | CANFD_FDF,
                    MAX_CANFD_DATA_LEN,
                )

            packet_id = int(can_id) & CAN_EFF_MASK
            return bridge.send_packet(bus, packet_id, bytes(data), 0, MAX_CAN_DATA_LEN)

        def can_read(
            slave_id: int, expected_can_id: int, expected_frames: int
        ) -> Tuple[int, bytes]:
            bridge = runtime.current_bridge()
            if bridge is None:
                return 0, bytes()

            bus = bridge.bus_for_slave(slave_id)
            if bridge.is_canfd_for_slave(slave_id):
                result = bridge.receive_canfd(
                    bus, expected_can_id, BXI_PCI_RX_WAIT_TIME_MS
                )
            else:
                result = bridge.receive_can(
                    bus,
                    expected_can_id,
                    expected_frames,
                    BXI_PCI_RX_WAIT_TIME_MS,
                )
            if result is None:
                return 0, bytes()
            return result

        _can_tx_callback = can_send
        _can_rx_callback = can_read
        libstark.set_can_tx_callback(_can_tx_callback)
        libstark.set_can_rx_callback(_can_rx_callback)
        _callbacks_installed = True


def setup_bxipci_can_callbacks() -> None:
    bridge = BxiPciRosRuntime.instance().current_bridge()
    if bridge is not None:
        bridge.set_default_protocol(False)
    setup_bxipci_callbacks()


def setup_bxipci_canfd_callbacks() -> None:
    bridge = BxiPciRosRuntime.instance().current_bridge()
    if bridge is not None:
        bridge.set_default_protocol(True)
    setup_bxipci_callbacks()


async def init_bxipci_device(
    bus: int = BXI_PCI_DEFAULT_BUS_INDEX,
    slave_id: int = 0x7E,
    *,
    master_id: int = 1,
    is_canfd: bool = True,
    hw_type: Any = None,
) -> BxiDeviceContext:
    _check_sdk()

    runtime = BxiPciRosRuntime.instance()
    bridge = runtime.ensure_bridge(bus)
    bridge.set_default_protocol(is_canfd)
    bridge.register_slave(slave_id, bus, is_canfd)
    setup_bxipci_callbacks()

    protocol = libstark.StarkProtocolType.CanFd if is_canfd else libstark.StarkProtocolType.Can
    if hw_type is None:
        hw_type = libstark.StarkHardwareType.Revo2Basic
    handle = libstark.init_device_handler(
        protocol,
        master_id=master_id,
        slave_id=slave_id,
        hw_type=hw_type,
    )
    runtime.register_device()
    return BxiDeviceContext(
        handle=handle,
        bus=int(bus),
        slave_id=int(slave_id) & 0xFF,
        is_canfd=is_canfd,
        protocol_type=protocol,
        hw_type=hw_type,
    )


async def cleanup_bxipci_device(ctx: Optional[BxiDeviceContext]) -> None:
    if ctx is not None:
        await ctx.close()


def stop_bxipci_runtime() -> None:
    BxiPciRosRuntime.instance().stop()
