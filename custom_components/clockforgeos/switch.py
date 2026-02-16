from __future__ import annotations

import asyncio
from time import monotonic

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
    "wakeOnMotionEnabled",
    "alarmEnable",
    "showTimeDate",
    "showTemperature",
    "showHumidity",
    "showPressure",
]

SWITCH_READ_ALIASES: dict[str, list[str]] = {
    "displayPower": ["displayPower"],
    "wakeOnMotionEnabled": ["wakeOnMotionEnabled"],
    "alarmEnable": ["alarmEnable", "enableAlarm"],
    "showTimeDate": ["showTimeDate", "enableTimeDisplay"],
    "showTemperature": ["showTemperature", "enableTempDisplay"],
    "showHumidity": ["showHumidity", "enableHumidDisplay"],
    "showPressure": ["showPressure", "enablePressDisplay"],
}

# For compatibility with older firmware variants that don't implement show* aliases.
SWITCH_WRITE_KEY: dict[str, str] = {
    "displayPower": "displayPower",
    "wakeOnMotionEnabled": "wakeOnMotionEnabled",
    "alarmEnable": "alarmEnable",
    "showTimeDate": "showTimeDate",
    "showTemperature": "showTemperature",
    "showHumidity": "showHumidity",
    "showPressure": "showPressure",
}

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
        self._pending_state: bool | None = None
        self._pending_until: float = 0.0
        self._last_known_state: bool | None = None

    @staticmethod
    def _prettify_name(key: str) -> str:
        # Convert camelCase or snake_case to Title Case
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
        s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
        return s2.replace('_', ' ').title()

    @property
    def is_on(self) -> bool:
        # Keep locally written state shortly to bridge firmware cache/read lag.
        if self._pending_state is not None and monotonic() < self._pending_until:
            return self._pending_state

        current_info = self.coordinator.data.get("current_info", {})
        config = self.coordinator.data.get("config", {})
        system_info = self.coordinator.data.get("system_info", {})

        for key in SWITCH_READ_ALIASES.get(self._key, [self._key]):
            if key in current_info:
                state = str(current_info.get(key)) not in ("0", "false", "False")
                self._last_known_state = state
                return state
            if key in config:
                state = str(config.get(key)) not in ("0", "false", "False")
                self._last_known_state = state
                return state
            if key in system_info:
                state = str(system_info.get(key)) not in ("0", "false", "False")
                self._last_known_state = state
                return state

        # Derive display power from manualDisplayOff if direct key is absent.
        if self._key == "displayPower":
            if "manualDisplayOff" in current_info:
                state = str(current_info.get("manualDisplayOff")) in ("0", "false", "False")
                self._last_known_state = state
                return state
            if "manualDisplayOff" in system_info:
                state = str(system_info.get("manualDisplayOff")) in ("0", "false", "False")
                self._last_known_state = state
                return state

        # Do not force OFF for unknown/missing keys.
        if self._last_known_state is not None:
            return self._last_known_state
        if self._pending_state is not None:
            return self._pending_state
        return False

    async def async_turn_on(self, **kwargs) -> None:
        await self._async_set_switch(True)

    async def async_turn_off(self, **kwargs) -> None:
        await self._async_set_switch(False)

    async def _async_set_switch(self, state: bool) -> None:
        self._pending_state = state
        self._pending_until = monotonic() + 5.0
        self._last_known_state = state
        self.async_write_ha_state()

        try:
            write_key = SWITCH_WRITE_KEY.get(self._key, self._key)
            await self.coordinator.api.save_setting(write_key, "true" if state else "false")
            # Backward compatibility for older firmware variants.
            if write_key == "showTimeDate":
                await self.coordinator.api.save_setting("enableTimeDisplay", "true" if state else "false")
            elif write_key == "showTemperature":
                await self.coordinator.api.save_setting("enableTempDisplay", "true" if state else "false")
            elif write_key == "showHumidity":
                await self.coordinator.api.save_setting("enableHumidDisplay", "true" if state else "false")
            elif write_key == "showPressure":
                await self.coordinator.api.save_setting("enablePressDisplay", "true" if state else "false")
        except ClockForgeOSApiError as err:
            self._pending_state = None
            self._pending_until = 0.0
            self.async_write_ha_state()
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        # Firmware caches /getCurrentInfos for ~1s; refresh after cache window.
        await asyncio.sleep(1.2)
        await self.coordinator.async_request_refresh()
        self._pending_state = None
        self._pending_until = 0.0
