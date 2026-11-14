"""Immutable MAVLink payloads, shared RC values, and telemetry state."""

from dataclasses import dataclass

@dataclass(frozen=True)
class Heartbeat:
    """HEARTBEAT (message ID 0).

    https://mavlink.io/en/messages/minimal.html#HEARTBEAT

    type: MAV_TYPE.
    autopilot: MAV_AUTOPILOT.
    base_mode: MAV_MODE_FLAG.
    custom_mode: unitless protocol value.
    system_status: MAV_STATE.
    mavlink_version: unitless protocol value.
    """

    type: int
    autopilot: int
    base_mode: int
    custom_mode: int
    system_status: int
    mavlink_version: int


@dataclass(frozen=True)
class Attitude:
    """ATTITUDE (message ID 30).

    https://mavlink.io/en/messages/common.html#ATTITUDE

    time_boot_ms: ms.
    roll: rad.
    pitch: rad.
    yaw: rad.
    rollspeed: rad/s.
    pitchspeed: rad/s.
    yawspeed: rad/s.
    """

    time_boot_ms: int
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float


@dataclass(frozen=True)
class VfrHud:
    """VFR_HUD (message ID 74).

    https://mavlink.io/en/messages/common.html#VFR_HUD

    airspeed: m/s.
    groundspeed: m/s.
    heading: deg.
    throttle: %.
    alt: m.
    climb: m/s.
    """

    airspeed: float
    groundspeed: float
    heading: int
    throttle: int
    alt: float
    climb: float


@dataclass(frozen=True)
class CommandLong:
    """COMMAND_LONG (message ID 76).

    https://mavlink.io/en/messages/common.html#COMMAND_LONG

    target_system: unitless protocol value.
    target_component: unitless protocol value.
    command: MAV_CMD.
    confirmation: unitless protocol value.
    param1: command-specific units.
    param2: command-specific units.
    param3: command-specific units.
    param4: command-specific units.
    param5: command-specific units.
    param6: command-specific units.
    param7: command-specific units.
    """

    target_system: int
    target_component: int
    command: int
    confirmation: int
    param1: float
    param2: float
    param3: float
    param4: float
    param5: float
    param6: float
    param7: float


@dataclass(frozen=True)
class CommandAck:
    """COMMAND_ACK (message ID 77).

    https://mavlink.io/en/messages/common.html#COMMAND_ACK

    command: MAV_CMD.
    result: MAV_RESULT.
    progress: unitless protocol value.
    result_param2: unitless protocol value.
    target_system: unitless protocol value.
    target_component: unitless protocol value.
    """

    command: int
    result: int
    progress: int = 0
    result_param2: int = 0
    target_system: int = 0
    target_component: int = 0


@dataclass(frozen=True)
class RcChannelsOverride:
    """RC_CHANNELS_OVERRIDE (message ID 70).

    https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE

    target_system: unitless protocol value.
    target_component: unitless protocol value.
    chan1_raw: us.
    chan2_raw: us.
    chan3_raw: us.
    chan4_raw: us.
    chan5_raw: us.
    chan6_raw: us.
    chan7_raw: us.
    chan8_raw: us.
    chan9_raw: us.
    chan10_raw: us.
    chan11_raw: us.
    chan12_raw: us.
    chan13_raw: us.
    chan14_raw: us.
    chan15_raw: us.
    chan16_raw: us.
    chan17_raw: us.
    chan18_raw: us.
    """

    target_system: int
    target_component: int
    chan1_raw: int
    chan2_raw: int
    chan3_raw: int
    chan4_raw: int
    chan5_raw: int
    chan6_raw: int
    chan7_raw: int
    chan8_raw: int
    chan9_raw: int = 0
    chan10_raw: int = 0
    chan11_raw: int = 0
    chan12_raw: int = 0
    chan13_raw: int = 0
    chan14_raw: int = 0
    chan15_raw: int = 0
    chan16_raw: int = 0
    chan17_raw: int = 0
    chan18_raw: int = 0


@dataclass(frozen=True)
class RcChannels:
    """Eight receiver-ordered RC values in microseconds, shared by MSP and MAVLink.

    MAVLink zero/release and ignore sentinels retain protocol-specific meaning.
    https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    """

    values: tuple[int, ...]

    def __post_init__(self) -> None:
        values = tuple(self.values)
        if len(values) != 8 or any(type(v) is not int or not 0 <= v <= 65535 for v in values):
            raise ValueError('Expected eight unsigned 16-bit RC channel values')
        object.__setattr__(self, 'values', values)


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Latest payloads and local monotonic reception times in seconds.

    Measured stream rates are in Hz. Missing payloads are None.
    """

    heartbeat: Heartbeat | None = None
    attitude: Attitude | None = None
    vfr_hud: VfrHud | None = None
    heartbeat_at: float = 0.0
    attitude_at: float = 0.0
    vfr_hud_at: float = 0.0
    vfr_hud_hz: float = 0.0
    attitude_hz: float = 0.0
