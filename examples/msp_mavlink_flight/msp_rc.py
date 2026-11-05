"""Send this simulation's RC channel mapping through the MSP library."""

from beta_client.msp import send_rc
from beta_client.transport import TcpTransport

from pwm import AUX_HIGH, AUX_LOW, CONTROL_CENTER


# UART1 is configured for MSP in sitl.config.
MSP_HOST = '127.0.0.1'
MSP_PORT = 5761


class MspRcSender:
    def __init__(self):
        self.transport = TcpTransport(MSP_HOST, MSP_PORT)

    def send(self, throttle: int, arm: bool) -> None:
        # AETR, AUX1 arms, AUX2 holds angle mode, remaining AUX channels stay low.
        send_rc(self.transport, [
            CONTROL_CENTER, CONTROL_CENTER, throttle, CONTROL_CENTER,
            AUX_HIGH if arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW,
        ])
        # Consume replies so continuous RC sending cannot fill receive buffers.
        self.transport.drain_responses()

    def close(self) -> None:
        self.transport.close()
