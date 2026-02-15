from __future__ import annotations

import asyncio

from homeassistant.components.switch import SwitchEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .api import ClockForgeOSApiError
from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity



BOOLEAN_SWITCH_KEYS = [
    "displayPower",
    "alarmEnable",
    "showTimeDate",
    "showTemperature",
    "showHumidity",
    "showPressure",
]

SWITCH_ICONS = {
    "displayPower": "mdi:monitor",
    "onboardLed": "mdi:led-on",
    "alarmEnable": "mdi:alarm",
    "mqttEnable": "mdi:lan",
    "debugEnabled": "mdi:bug",
    "enableBlink": "mdi:flash",
    "enableDST": "mdi:weather-sunny-clock",
    "enableAutoShutoff": "mdi:power-sleep",
    "tubesSleep": "mdi:power-sleep",
    "wakeOnMotionEnabled": "mdi:run-fast",
    "manualDisplayOff": "mdi:monitor-off",
    "enableTimeDisplay": "mdi:clock",
    "enableTempDisplay": "mdi:thermometer",
    "enableHumidDisplay": "mdi:water-percent",
    "enablePressDisplay": "mdi:gauge",
    "showPressure": "mdi:gauge",
    "enableDoubleBlink": "mdi:flash-alert",
    "enableRadar": "mdi:radar",
}

async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    switches = [
        ClockForgeOSSettingSwitch(coordinator, entry, key)
        for key in BOOLEAN_SWITCH_KEYS
    ]
    async_add_entities(switches)



class ClockForgeOSSettingSwitch(ClockForgeOSEntity, SwitchEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry, key: str) -> None:
        super().__init__(coordinator)
        self._key = key
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = self._prettify_name(key)
        self._attr_icon = SWITCH_ICONS.get(key)

    @staticmethod
    def _prettify_name(key: str) -> str:
        # Convert camelCase or snake_case to Title Case
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
        s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
        return s2.replace('_', ' ').title()

    @property
    def is_on(self) -> bool:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        value = current_info.get(self._key, system_info.get(self._key, 0))
        return str(value) not in ("0", "false", "False")

    async def async_turn_on(self, **kwargs) -> None:
        await self._async_set_switch(True)

    async def async_turn_off(self, **kwargs) -> None:
        await self._async_set_switch(False)

    async def _async_set_switch(self, state: bool) -> None:
        try:
            await self.coordinator.api.save_setting(self._key, "true" if state else "false")
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        # Firmware caches /getCurrentInfos for ~1s; refresh after cache window.
        await asyncio.sleep(1.2)
        await self.coordinator.async_request_refresh()
