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

    async def _async_update_data(self) -> dict[str, Any]:
        try:
            system_info = await self.api.get_system_info()
            current_info = await self.api.get_current_info()
            return {
                "system_info": system_info,
                "current_info": current_info,
            }
        except ClockForgeOSApiError as err:
            raise UpdateFailed(str(err)) from err
