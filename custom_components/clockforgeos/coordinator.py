from __future__ import annotations

import logging
from datetime import timedelta
from typing import Any

from homeassistant.core import HomeAssistant
from homeassistant.helpers.aiohttp_client import async_get_clientsession
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .api import ClockForgeOSApi, ClockForgeOSApiError
from .const import DOMAIN


class ClockForgeOSCoordinator(DataUpdateCoordinator[dict[str, Any]]):
    """Coordinator for ClockForgeOS data."""

    def __init__(
        self,
        hass: HomeAssistant,
        host: str,
        update_interval: timedelta,
        password: str | None = None,
    ) -> None:
        super().__init__(
            hass,
            logger=logging.getLogger(__name__),
            name=DOMAIN,
            update_interval=update_interval,
        )
        self.host = host
        self.api = ClockForgeOSApi(async_get_clientsession(hass), host, password)
        self._last_uptime_minutes: float | None = None

    @staticmethod
    def _extract_uptime_minutes(current_info: dict[str, Any], system_info: dict[str, Any]) -> float | None:
        for src in (current_info, system_info):
            raw = src.get("uptimeMinutes")
            if raw is None:
                continue
            try:
                return float(raw)
            except (TypeError, ValueError):
                continue
        return None

    async def _async_update_data(self) -> dict[str, Any]:
        try:
            was_available = self.last_update_success
            system_info = await self.api.get_system_info()
            current_info = await self.api.get_current_info()
            uptime_minutes = self._extract_uptime_minutes(current_info, system_info)
            reboot_detected = (
                self._last_uptime_minutes is not None
                and uptime_minutes is not None
                and (uptime_minutes + 1.0) < self._last_uptime_minutes
            )
            # On reconnect (after downtime), refresh auth once so config can be
            # pulled automatically without waiting for a write action.
            if not was_available and self.api.has_password:
                try:
                    await self.api.authenticate()
                except ClockForgeOSApiError:
                    pass
            # Also refresh auth automatically right after device reboot so
            # protected config/state recovers without manual button press.
            if (
                self.api.has_password
                and not self.api.has_valid_token
                and (
                    reboot_detected
                    or (uptime_minutes is not None and uptime_minutes <= 2.0)
                )
            ):
                try:
                    await self.api.authenticate()
                except ClockForgeOSApiError:
                    pass
            config = await self.api.get_configuration()
            self._last_uptime_minutes = uptime_minutes
            return {
                "system_info": system_info,
                "current_info": current_info,
                "config": config,
            }
        except ClockForgeOSApiError as err:
            raise UpdateFailed(str(err)) from err
