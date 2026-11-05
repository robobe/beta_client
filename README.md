# Betaflight client examples

Examples for exploring Betaflight SITL, MAVLink telemetry and simulated flight.

## Dependencies

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and use Python 3.12 or newer. From the repository root, run:

```sh
uv sync
```

This creates or updates the `.venv` environment using `pyproject.toml` and `uv.lock`, including `pymavlink` and `pyserial`. Run an example in that environment with `uv run`, for example:

```sh
uv run python examples/mavlink_heartbeat.py tcp:127.0.0.1:5762
```

For the flight examples, also install the `flight` extra:

```sh
uv sync --extra flight
```

This installs `bullet_drone` and its dependencies. The flight extra points to `/home/user/workspaces/bullet/bullet_drone` in `pyproject.toml`; change that path if your Bullet checkout is elsewhere. To keep the extra when running a command through `uv run`, include `--extra flight` in that command as well. The MSP flight task uses this project’s `flight` extra. The pure-MAVLink flight task still uses the separate Bullet workspace environment.

After changing dependencies, run `uv sync` again to update the lockfile and environment. Use `uv sync --locked` when you want synchronization to fail if `uv.lock` needs updating.

## Examples

| Example | Summary |
| --- | --- |
| [MAVLink heartbeat](examples/mavlink_heartbeat.py) | Connect to a MAVLink TCP endpoint and print the first heartbeat. |
| [MAVLink rates](examples/mavlink_rates.py) | Request attitude and VFR HUD messages at 50 Hz and measure their received rates over TCP or serial. |
| [USB MAVLink monitor](examples/mavlink_usb_monitor.py) | Print MAVLink messages and per-type rates from a serial adapter. |
| [USB MAVLink rates](examples/mavlink_usb_rates.py) | Measure attitude and VFR HUD rates through a serial adapter. |
| [MSP and MAVLink flight](examples/msp_mavlink_flight/README.md) | Experimental simulation: send RC commands over MSP and read telemetry over MAVLink. Requires the separate `bullet_drone` project. |
| [MAVLink flight](examples/mavlink_flight/README.md) | Standalone pure-MAVLink flight experiment; **not working**, pending investigation of MAVLink RC receiver support and configuration. |

## VS Code tasks

Run these using **Terminal → Run Task**.

| Task | Function |
| --- | --- |
| `SITL: Configure MAVLink UART2` | Saves the MAVLink telemetry configuration in the `.sitl` working directory. Stop SITL before running it. |
| `SITL: Run` | Starts the bundled SITL using the saved `.sitl` configuration. |
| `MAVLink: Read heartbeat` | Connects to an already running SITL on port 5762 and prints a heartbeat. |
| `SITL: Run and choose MAVLink example` | Starts SITL, then runs the selected heartbeat or message-rate example. Configure UART2 first. |
| `Bullet: SITL MAVLink flight` | Starts the standalone MAVLink flight experiment and its SITL. **Not working** with the bundled firmware. |
| `Bullet: SITL MSP MAVLink flight` | Starts the MSP control and MAVLink telemetry flight example and its SITL. |

The MSP flight task requires the `bullet_drone` path configured in `pyproject.toml`. The pure-MAVLink flight task uses `/home/user/workspaces/bullet`; update its command if the workspace is installed elsewhere.

## Betaflight settings

The telemetry examples require a firmware build with MAVLink telemetry enabled and a UART assigned to MAVLink. For the included SITL binary, [config/sitl-mavlink.txt](config/sitl-mavlink.txt) enables telemetry and assigns MAVLink telemetry to UART2 (TCP port 5762). UART1 is typically used for MSP on TCP port 5761. SITL saves settings in its working directory, so configure the instance you intend to run.

The MSP flight example uses its own [SITL configuration](examples/msp_mavlink_flight/sitl.config): MSP receiver input, AETR channel order, AUX1 for arming and AUX2 for angle mode. Its Bullet integration requires a compatible sensor-only SITL build. Both flight examples are experimental and intended for simulation only.

- [Build SITL with MAVLink telemetry and configure UART2](docs/build-betaflight-mavlink.md#build-sitl-locally)
- [Build and flash hardware firmware with MAVLink telemetry](docs/build-betaflight-mavlink.md#build-and-flash-hardware-firmware)
- [Betaflight and MAVLink messages and configuration](docs/betaflight-mavlink.md)
- [Betaflight MSP and SITL](docs/betaflight-msp.md)

MAVLink telemetry support alone does not establish that MAVLink RC commands are supported or configured. The bundled binary rejects the receiver setting used by the pure-MAVLink example; that integration needs further investigation.

## MSP RC client

Build and send eight RC channels in the receiver's configured order over TCP:

```python
from beta_client.msp import send_rc
from beta_client.transport import TcpTransport

with TcpTransport('127.0.0.1', 5761) as transport:
    send_rc(transport, [1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000])
    transport.drain_responses()  # Discard replies in a send-only RC loop.
```

For serial, use `SerialTransport('/dev/ttyUSB0', 115200)` in the same way, with the device and baud rate configured for your receiver. Use `build_rc_message(channels)` when you need the encoded frame separately.

The caller owns the connection and must send values continuously at a rate appropriate for the receiver and its failsafe. A single frame does not provide sustained control. Test in simulation first.

TCP and serial writes block with a default 20 ms timeout, matching one 50 Hz control period. Override `timeout` to fit your loop budget; TCP connection establishment has a separate `connect_timeout` default of 3 seconds. Serial partial writes share one timeout budget. These are host-side write budgets, not guaranteed delivery or real-time deadlines.

Call `drain_responses()` each cycle in a send-only loop to discard buffered replies without waiting. Each call discards at most 4096 bytes to bound receive work. Omit draining if you need to parse replies instead. Transports are intended for a single caller; do not read or write them concurrently.

## Threaded MAVLink telemetry

`MavlinkTelemetry` receives on its own thread; callers read frozen snapshots without connection I/O:

```python
from beta_client.mavlink import MavlinkTelemetry
from pymavlink import mavutil

with MavlinkTelemetry('tcp:127.0.0.1:5762') as telemetry:
    telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, rate_hz=50)
    sample = telemetry.snapshot()
    print(sample.altitude, sample.altitude_at, sample.armed)
```

For serial, pass a device endpoint such as `/dev/ttyUSB0` and `baud=57600`. Context entry waits only for a vehicle heartbeat. Call `set_message_rate(message_id, rate_hz, timeout=5.0)` to request each desired stream. It blocks until the selected vehicle acknowledges acceptance, raises `RuntimeError` on rejection, and raises `TimeoutError` if no final acknowledgement arrives. Only finite positive rates are supported; actual delivery rates may differ. Concurrent requests are serialized, with the ACK timeout starting when the worker sends the command. A timeout fails and closes the link, preventing a late acknowledgement from being attributed to another request. Snapshot fields can be `None` until received; timestamps use local monotonic time. Consumers must check freshness and validity before control. Worker failures raise from `snapshot()` with the original cause; reconnection is not automatic. The worker owns the connection and closes it on exit. Protocol waits are bounded, but initial connection opening is subject to pymavlink and operating-system behavior.

See the [flight control flow](examples/msp_mavlink_flight/README.md#threaded-telemetry-and-altitude-control) for how altitude drives MSP throttle commands.
