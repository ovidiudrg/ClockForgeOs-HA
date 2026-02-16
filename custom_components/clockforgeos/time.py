from __future__ import annotations

import asyncio
from datetime import time

from homeassistant.components.time import TimeEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .api import ClockForgeOSApiError
from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity


def _parse_hhmm(value: str | None) -> time | None:
    if not value or ":" not in value:
        return None
    try:
        hh, mm = value.split(":", 1)
        return time(hour=int(hh), minute=int(mm))
    except (TypeError, ValueError):
        return None


class ClockForgeOSAlarmTime(ClockForgeOSEntity, TimeEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = f"{entry.entry_id}_alarm_time"
        self._attr_name = "Alarm Time"
        self._attr_icon = "mdi:alarm"

    @property
    def native_value(self) -> time | None:
        current_info = self.coordinator.data.get("current_info", {})
        config = self.coordinator.data.get("config", {})
        system_info = self.coordinator.data.get("system_info", {})
        raw = current_info.get("alarmTime", config.get("alarmTime", system_info.get("alarmTime")))
        return _parse_hhmm(str(raw) if raw is not None else None)

    async def async_set_value(self, value: time) -> None:
        try:
            await self.coordinator.api.save_setting("alarmTimeHours", str(value.hour))
            await self.coordinator.api.save_setting("alarmTimeMinutes", str(value.minute))
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set alarm time: {err}") from err
        await asyncio.sleep(1.2)
        await self.coordinator.async_request_refresh()


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    async_add_entities([ClockForgeOSAlarmTime(coordinator, entry)])
