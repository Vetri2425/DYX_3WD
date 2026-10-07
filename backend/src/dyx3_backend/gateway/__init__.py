"""Client side of the dyx3_system_gateway socket protocol (docs/contracts/dyx3_system_gateway.md)."""

from dyx3_backend.gateway.client import (
    GatewayClient,
    GatewayError,
    GatewayTimeout,
    GatewayUnavailable,
)

__all__ = ["GatewayClient", "GatewayError", "GatewayTimeout", "GatewayUnavailable"]
