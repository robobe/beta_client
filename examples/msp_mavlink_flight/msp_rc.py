"""Send this simulation's RC channel mapping through the MSP library."""

from beta_client.msp import send_rc
from beta_client.transport import TcpTransport

from pwm import AUX_HIGH, AUX_LOW, CONTROL_CENTER


class MspRcSender:
    def __init__(self):
        self.transport = TcpTransport('127.0.0.1', 5761)

    def send(self, throttle: int, arm: bool) -> None:
        send_rc(self.transport, [
            CONTROL_CENTER, CONTROL_CENTER, throttle, CONTROL_CENTER,
            AUX_HIGH if arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW,
        ])
        # Consume replies so continuous RC sending cannot fill receive buffers.
        self.transport.drain_responses()

    def close(self) -> None:
        self.transport.close()
