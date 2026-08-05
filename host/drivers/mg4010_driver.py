"""High-level MG4010 motor driver using protocol-native motor units."""

from __future__ import annotations

import can

from drivers.can_bus import (
    CanMotorBus,
    CanRequestNotSentError,
    MotorCommunicationError,
)
from drivers.mg4010_protocol import (
    MotorError,
    MotorFault,
    MotorProtocolError,
    MotorSingleTurnPosition,
    MotorStatus,
    build_position_command_2,
    build_motor_off_request,
    build_motor_run_request,
    build_read_fault_request,
    build_read_multi_turn_request,
    build_read_single_turn_request,
    build_read_status_request,
    build_request_id,
    build_response_id,
    build_stop_request,
    parse_fault_response,
    parse_multi_turn_response,
    parse_position_command_2_response,
    parse_single_turn_response,
    parse_status_response,
)


class MotorCommandResultUnknownError(MotorCommunicationError):
    """An issued motion command could not be confirmed by its response."""


class MG4010Driver:
    """Control one MG4010 through a shared :class:`CanMotorBus`.

    Angles and speeds exposed here are motor-side degrees and degrees/second.
    Gear ratios, joint zeros, directions, and joint limits belong to the joint
    layer and intentionally do not appear in this class.
    """

    def __init__(self, bus: CanMotorBus, motor_id: int) -> None:
        self.bus = bus
        self.motor_id = motor_id
        # Building both IDs validates the motor ID once during construction.
        self._request_id = build_request_id(motor_id)
        self._response_id = build_response_id(motor_id)

    @property
    def request_id(self) -> int:
        """CAN arbitration ID used for requests to this motor."""

        return self._request_id

    @property
    def response_id(self) -> int:
        """Protocol-defined CAN arbitration ID used by this motor's replies."""

        return self._response_id

    def read_single_turn_position(self) -> MotorSingleTurnPosition:
        """Read the 0x94 absolute encoder cycle position."""

        response = self._transact(build_read_single_turn_request())
        return parse_single_turn_response(bytes(response.data))

    def read_multi_turn_position_deg(self) -> float:
        """Read the current power-cycle 0x92 motor multi-turn coordinate."""

        response = self._transact(build_read_multi_turn_request())
        return parse_multi_turn_response(bytes(response.data)).motor_deg

    def read_status(self) -> MotorStatus:
        """Read the 0x9C temperature, speed, current, and encoder status."""

        response = self._transact(build_read_status_request())
        return parse_status_response(bytes(response.data))

    def read_fault(self) -> MotorFault:
        """Read the 0x9A motor state and fault information."""

        response = self._transact(build_read_fault_request())
        return parse_fault_response(bytes(response.data))

    def command_position(
        self,
        target_motor_deg: float,
        max_motor_speed_deg_s: float,
    ) -> None:
        """Submit a 0xA4 target and return after its CAN reply is confirmed.

        This method does not wait for mechanical arrival.  If the command was
        submitted but no valid response can be confirmed, one 0x81 stop is
        attempted before :class:`MotorCommandResultUnknownError` is raised.
        """

        # Validate and encode before entering the recovery region.  A local
        # argument error means no A4 was sent and therefore must not send stop.
        payload = build_position_command_2(
            target_motor_deg=target_motor_deg,
            max_motor_speed_deg_s=max_motor_speed_deg_s,
        )
        try:
            response = self._transact(payload)
            parse_position_command_2_response(bytes(response.data))
        except CanRequestNotSentError:
            # No A4 send was attempted, so the mechanical result is not unknown
            # and an unsolicited stop would be misleading.
            raise
        except MotorError as command_error:
            stop_error: MotorError | None = None
            try:
                # The recovery contract is one best-effort stop transmission.
                # Do not use transact() here because its configured retries could
                # issue more than one 0x81 frame.
                self.bus.send_only(self.request_id, build_stop_request())
            except MotorError as exc:
                stop_error = exc

            detail = "a 0x81 stop command was attempted"
            if stop_error is not None:
                detail += f", but that stop attempt also failed: {stop_error}"
            raise MotorCommandResultUnknownError(
                f"motor ID {self.motor_id}: the 0xA4 position command may have "
                f"been received, but its response could not be confirmed and "
                f"the final mechanical state is unknown; {detail}"
            ) from command_error

    def stop(self) -> None:
        """Submit the repeat-safe 0x81 software stop command."""

        self._transact(build_stop_request())

    def enable(self) -> None:
        """Submit 0x88 and wait for the protocol echo response."""

        self._transact(build_motor_run_request())

    def disable(self) -> None:
        """Submit 0x80 motor-off and wait for the protocol echo response."""

        self._transact(build_motor_off_request())

    def _transact(self, payload: bytes) -> can.Message:
        """Run one request/reply transaction through the shared bus."""

        return self.bus.transact(
            arbitration_id=self.request_id,
            data=payload,
            expected_response_id=self.response_id,
            expected_command=payload[0],
        )


__all__ = [
    "MG4010Driver",
    "MotorCommandResultUnknownError",
    "MotorCommunicationError",
    "MotorError",
    "MotorProtocolError",
]
