"""Thread-safe request/response transport for MG4010 CAN motors."""

from __future__ import annotations

from collections.abc import Callable
import platform
import threading
import time
from typing import Any, Protocol

import can

from drivers.mg4010_protocol import FRAME_DLC, MotorError, MotorProtocolError


DEFAULT_BITRATE = 1_000_000
DEFAULT_TIMEOUT_SECONDS = 0.1
DEFAULT_RETRIES = 2
_MAX_DRAINED_FRAMES = 1_024


class MotorCommunicationError(MotorError):
    """Base error for CAN transport and transaction failures."""


class CanRequestNotSentError(MotorCommunicationError):
    """A transaction failed before any request send was attempted."""


class CanBusNotOpenError(CanRequestNotSentError):
    """A transaction was requested before the CAN bus was opened."""


class CanTransactionTimeoutError(MotorCommunicationError):
    """No matching response arrived before all attempts expired."""


class CanFrameValidationError(MotorProtocolError):
    """A response used the expected ID but violated the CAN frame contract."""


class CanBusLike(Protocol):
    """The small python-can surface used by :class:`CanMotorBus`."""

    def send(self, msg: can.Message, timeout: float | None = None) -> None: ...

    def recv(self, timeout: float | None = None) -> can.Message | None: ...

    def shutdown(self) -> None: ...


RawFrameCallback = Callable[[str, can.Message], None]


class CanMotorBus:
    """Own and serialize access to a CAN bus shared by multiple motors.

    The protocol response ID is the only accepted ID by default.  Set
    ``allow_same_id_response`` explicitly for motor firmware known to reply on
    the request ID; transmit echoes are still excluded via ``is_rx``.

    ``bus`` is intended for tests and other already-created python-can
    transports.  It is treated as open and is shut down by :meth:`close`.
    ``gs_usb_device`` accepts an already VID/PID-resolved gs_usb handle; opening
    then targets its current USB bus/address instead of selecting scan index 0.
    """

    def __init__(
        self,
        interface: str | None = None,
        channel: int | str | None = None,
        bitrate: int | None = None,
        *,
        timeout: float = DEFAULT_TIMEOUT_SECONDS,
        retries: int = DEFAULT_RETRIES,
        allow_same_id_response: bool = False,
        raw_frame_callback: RawFrameCallback | None = None,
        bus: CanBusLike | None = None,
        gs_usb_device: object | None = None,
    ) -> None:
        if timeout <= 0:
            raise ValueError("timeout must be greater than zero")
        if retries < 0:
            raise ValueError("retries must be zero or greater")
        if bitrate is not None and bitrate <= 0:
            raise ValueError("bitrate must be greater than zero")
        if not isinstance(allow_same_id_response, bool):
            raise TypeError("allow_same_id_response must be a bool")

        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.timeout = timeout
        self.retries = retries
        self.allow_same_id_response = allow_same_id_response
        self.raw_frame_callback = raw_frame_callback
        self.gs_usb_device = gs_usb_device

        self._bus: CanBusLike | None = bus
        self._lock = threading.RLock()

    def __enter__(self) -> CanMotorBus:
        self.open()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        """Whether a python-can transport is currently attached."""

        with self._lock:
            return self._bus is not None

    def open(self) -> None:
        """Open the configured gs_usb or SocketCAN transport once."""

        with self._lock:
            if self._bus is not None:
                return

            interface, channel, bitrate = self._resolved_connection()
            kwargs: dict[str, Any] = {
                "interface": interface,
                "channel": channel,
                "ignore_config": True,
            }

            if interface == "gs_usb":
                if not isinstance(channel, int):
                    raise ValueError("gs_usb channel must be an integer adapter index")
                if bitrate is None:
                    raise ValueError("gs_usb requires a bitrate")

                # python-can may leave a partially initialized gs_usb object when an
                # adapter index is absent.  Preflight with the already-proven scan API
                # and do not use detect_available_configs().
                try:
                    from gs_usb.gs_usb import GsUsb
                except ImportError as exc:
                    raise can.CanInitializationError(
                        "gs_usb is required for interface 'gs_usb'; "
                        "install host/requirements-macos.txt"
                    ) from exc

                if self.gs_usb_device is None:
                    devices = GsUsb.scan()
                    if channel < 0 or channel >= len(devices):
                        raise can.CanInitializationError(
                            f"Cannot find gs_usb device index {channel}. "
                            f"Devices found: {len(devices)}"
                        )
                else:
                    device_bus = getattr(self.gs_usb_device, "bus", None)
                    device_address = getattr(self.gs_usb_device, "address", None)
                    if device_bus is None or device_address is None:
                        raise can.CanInitializationError(
                            "resolved gs_usb device requires bus and address metadata"
                        )
                    kwargs["bus"] = device_bus
                    kwargs["address"] = device_address
                kwargs["bitrate"] = bitrate
            elif interface != "socketcan":
                raise ValueError(
                    f"unsupported CAN interface {interface!r}; "
                    "expected 'gs_usb' or 'socketcan'"
                )

            # Assign only after successful construction so close() never sees a
            # half-created transport.
            self._bus = can.Bus(**kwargs)
            self.interface = interface
            self.channel = channel
            self.bitrate = bitrate

    def close(self) -> None:
        """Shut down the transport; repeated calls are safe."""

        with self._lock:
            bus = self._bus
            if bus is None:
                return
            # Clear first so a failed shutdown still leaves this object closed and a
            # second call cannot shut down the same backend twice.
            self._bus = None
            try:
                bus.shutdown()
            except Exception as exc:  # python-can backends use several error types
                raise MotorCommunicationError(
                    f"failed to close CAN interface {self.interface!r} "
                    f"channel {self.channel!r}: {exc}"
                ) from exc

    def transact(
        self,
        arbitration_id: int,
        data: bytes,
        expected_response_id: int,
        expected_command: int,
        expected_param_id: int | None = None,
    ) -> can.Message:
        """Send one request and return its validated response.

        Queue draining, every retry, and response matching all happen under the
        same lock, preventing one motor driver from consuming another's frames.
        """

        payload = self._validate_request(
            arbitration_id,
            data,
            expected_response_id=expected_response_id,
            expected_command=expected_command,
            expected_param_id=expected_param_id,
        )
        request = can.Message(
            arbitration_id=arbitration_id,
            data=payload,
            is_extended_id=False,
            is_remote_frame=False,
            is_error_frame=False,
            is_fd=False,
            check=True,
        )

        with self._lock:
            bus = self._require_bus()
            accepted_ids = {expected_response_id}
            if self.allow_same_id_response:
                accepted_ids.add(arbitration_id)

            last_protocol_error: MotorProtocolError | None = None
            request_was_sent = False
            attempts = self.retries + 1
            for _attempt in range(attempts):
                try:
                    self._drain_receive_queue(bus)
                except MotorCommunicationError as exc:
                    if not request_was_sent:
                        raise CanRequestNotSentError(
                            f"command 0x{expected_command:02X} was not sent because "
                            f"the receive queue could not be prepared: {exc}"
                        ) from exc
                    raise
                self._emit_raw_frame("TX", request)
                try:
                    bus.send(request)
                except Exception as exc:
                    raise MotorCommunicationError(
                        f"failed to send command 0x{expected_command:02X} on "
                        f"CAN ID 0x{arbitration_id:03X}: {exc}"
                    ) from exc
                request_was_sent = True

                deadline = time.monotonic() + self.timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        response = bus.recv(remaining)
                    except Exception as exc:
                        raise MotorCommunicationError(
                            f"failed while receiving command 0x{expected_command:02X} "
                            f"response: {exc}"
                        ) from exc
                    if response is None:
                        break

                    self._emit_raw_frame(
                        "RX" if response.is_rx else "RX-ECHO", response
                    )
                    # Ignore adapter TX echoes and all traffic for other motors.
                    if response.is_rx is False:
                        continue
                    if response.arbitration_id not in accepted_ids:
                        continue

                    try:
                        self._validate_response(
                            response,
                            expected_command=expected_command,
                            expected_param_id=expected_param_id,
                        )
                    except MotorProtocolError as exc:
                        # A delayed response for an earlier command can have the same
                        # ID. Keep listening, but preserve the useful validation error
                        # if no current response follows.
                        last_protocol_error = exc
                        continue
                    return response

            if last_protocol_error is not None:
                raise last_protocol_error

            accepted_text = ", ".join(
                f"0x{response_id:03X}" for response_id in sorted(accepted_ids)
            )
            raise CanTransactionTimeoutError(
                f"no matching response for command 0x{expected_command:02X} "
                f"from CAN ID(s) {accepted_text} after {attempts} attempt(s)"
            )

    def send_only(self, arbitration_id: int, data: bytes) -> None:
        """Send one standard eight-byte CAN data frame without awaiting a reply."""

        payload = self._validate_outgoing_frame(arbitration_id, data)
        request = can.Message(
            arbitration_id=arbitration_id,
            data=payload,
            is_extended_id=False,
            is_remote_frame=False,
            is_error_frame=False,
            is_fd=False,
            check=True,
        )
        with self._lock:
            bus = self._require_bus()
            self._emit_raw_frame("TX", request)
            try:
                bus.send(request)
            except Exception as exc:
                raise MotorCommunicationError(
                    f"failed to send frame on CAN ID 0x{arbitration_id:03X}: {exc}"
                ) from exc

    def _resolved_connection(self) -> tuple[str, int | str, int | None]:
        interface = self.interface
        channel = self.channel
        bitrate = self.bitrate

        if interface is None:
            system_name = platform.system().lower()
            if self.gs_usb_device is not None or system_name == "darwin":
                interface = "gs_usb"
            elif system_name == "linux":
                interface = "socketcan"
            else:
                raise ValueError(
                    f"unsupported platform {platform.system()!r}; provide interface"
                )

        if channel is None:
            channel = 0 if interface == "gs_usb" else "can0"
        if interface == "gs_usb":
            if isinstance(channel, str):
                try:
                    channel = int(channel, 0)
                except ValueError as exc:
                    raise ValueError(
                        "gs_usb channel must be an integer adapter index"
                    ) from exc
            if bitrate is None:
                bitrate = DEFAULT_BITRATE
        elif interface == "socketcan":
            if self.gs_usb_device is not None:
                raise ValueError(
                    "gs_usb_device cannot be used with interface 'socketcan'"
                )
            # SocketCAN bitrate belongs to the OS network interface configuration.
            bitrate = None

        return interface, channel, bitrate

    def _require_bus(self) -> CanBusLike:
        if self._bus is None:
            raise CanBusNotOpenError(
                "CAN bus is not open; call open() or use CanMotorBus as a context manager"
            )
        return self._bus

    def _drain_receive_queue(self, bus: CanBusLike) -> None:
        for _ in range(_MAX_DRAINED_FRAMES):
            try:
                stale = bus.recv(0.0)
            except Exception as exc:
                raise MotorCommunicationError(
                    f"failed to clear the CAN receive queue: {exc}"
                ) from exc
            if stale is None:
                return
            self._emit_raw_frame(
                "RX-STALE" if stale.is_rx else "RX-ECHO-STALE", stale
            )
        raise MotorCommunicationError(
            "CAN receive queue did not become empty after "
            f"{_MAX_DRAINED_FRAMES} frames"
        )

    def _emit_raw_frame(self, direction: str, message: can.Message) -> None:
        if self.raw_frame_callback is not None:
            self.raw_frame_callback(direction, message)

    @staticmethod
    def _validate_outgoing_frame(arbitration_id: int, data: bytes) -> bytes:
        if not isinstance(arbitration_id, int) or isinstance(arbitration_id, bool):
            raise TypeError("arbitration_id must be an int")
        if not 0 <= arbitration_id <= 0x7FF:
            raise ValueError("arbitration_id must fit a standard 11-bit CAN ID")
        try:
            payload = bytes(data)
        except (TypeError, ValueError) as exc:
            raise TypeError("data must be bytes-like") from exc
        if len(payload) != FRAME_DLC:
            raise ValueError(f"CAN data must contain exactly {FRAME_DLC} bytes")
        return payload

    def _validate_request(
        self,
        arbitration_id: int,
        data: bytes,
        *,
        expected_response_id: int,
        expected_command: int,
        expected_param_id: int | None,
    ) -> bytes:
        payload = self._validate_outgoing_frame(arbitration_id, data)
        if not isinstance(expected_response_id, int) or isinstance(
            expected_response_id, bool
        ):
            raise TypeError("expected_response_id must be an int")
        if not 0 <= expected_response_id <= 0x7FF:
            raise ValueError(
                "expected_response_id must fit a standard 11-bit CAN ID"
            )
        if (
            expected_response_id == arbitration_id
            and not self.allow_same_id_response
        ):
            raise ValueError(
                "same-ID responses require allow_same_id_response=True"
            )
        if not isinstance(expected_command, int) or isinstance(expected_command, bool):
            raise TypeError("expected_command must be an int")
        if not 0 <= expected_command <= 0xFF:
            raise ValueError("expected_command must fit one byte")
        if payload[0] != expected_command:
            raise ValueError(
                f"request command 0x{payload[0]:02X} does not match expected "
                f"command 0x{expected_command:02X}"
            )
        if expected_param_id is not None:
            if not isinstance(expected_param_id, int) or isinstance(
                expected_param_id, bool
            ):
                raise TypeError("expected_param_id must be an int or None")
            if not 0 <= expected_param_id <= 0xFF:
                raise ValueError("expected_param_id must fit one byte")
        return payload

    @staticmethod
    def _validate_response(
        message: can.Message,
        *,
        expected_command: int,
        expected_param_id: int | None,
    ) -> None:
        if (
            message.is_extended_id
            or message.is_remote_frame
            or message.is_error_frame
            or message.is_fd
        ):
            raise CanFrameValidationError(
                "expected a standard CAN 2.0 data frame; got "
                f"extended={message.is_extended_id}, "
                f"remote={message.is_remote_frame}, "
                f"error={message.is_error_frame}, fd={message.is_fd}"
            )
        if message.dlc != FRAME_DLC or len(message.data) != FRAME_DLC:
            raise CanFrameValidationError(
                f"expected DLC/data length {FRAME_DLC}, got DLC={message.dlc}, "
                f"data length={len(message.data)}"
            )
        if message.data[0] != expected_command:
            raise CanFrameValidationError(
                f"expected command 0x{expected_command:02X}, "
                f"got 0x{message.data[0]:02X}"
            )
        if expected_param_id is not None and message.data[1] != expected_param_id:
            raise CanFrameValidationError(
                f"expected ParamID 0x{expected_param_id:02X}, "
                f"got 0x{message.data[1]:02X}"
            )


__all__ = [
    "CanBusNotOpenError",
    "CanFrameValidationError",
    "CanMotorBus",
    "CanRequestNotSentError",
    "CanTransactionTimeoutError",
    "MotorCommunicationError",
]
