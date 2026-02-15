from __future__ import annotations

import asyncio

from homeassistant.components.select import SelectEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.exceptions import HomeAssistantError

from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity
from .api import ClockForgeOSApiError

RGB_EFFECT_LABELS = {
    "0": "OFF",
    "1": "Fixed Color",
    "2": "Rainbow Flow",
    "3": "Rainbow Stepper",
    "4": "Color Dimmer",
    "5": "Color Changer",
    "6": "Color FlowChanger",
    "7": "Color FlowChanger Random-1",
    "8": "Color FlowChanger Random-2",
    "9": "Color FlowChanger Random-3",
    "10": "Knight Rider's KITT",
    "11": "HeartBeat",
}

# List of all enumerated /saveSetting keys to expose as selects
SELECT_KEYS = {
    "rgbEffect": list(RGB_EFFECT_LABELS.values()),
}

async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    selects = [
        ClockForgeOSSettingSelect(coordinator, entry, key, options)
        for key, options in SELECT_KEYS.items()
    ]
    async_add_entities(selects)

class ClockForgeOSSettingSelect(ClockForgeOSEntity, SelectEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry, key: str, options: list[str]) -> None:
        super().__init__(coordinator)
        self._key = key
        self._options = options
        self._label_to_value = {v: k for k, v in RGB_EFFECT_LABELS.items()} if key == "rgbEffect" else {}
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = self._prettify_name(key)
        self._attr_options = options

    @staticmethod
    def _prettify_name(key: str) -> str:
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
        s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
        return s2.replace('_', ' ').title()

    @property
    def current_option(self) -> str | None:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        value = current_info.get(self._key, system_info.get(self._key))
        if value is None:
            return None
        value_str = str(value)
        if self._key == "rgbEffect":
            if value_str in RGB_EFFECT_LABELS:
                return RGB_EFFECT_LABELS[value_str]
            if value_str in self._options:
                return value_str
            return None
        return value_str

    async def async_select_option(self, option: str) -> None:
        try:
            payload = option
            if self._key == "rgbEffect":
                payload = self._label_to_value.get(option, option)
            await self.coordinator.api.save_setting(self._key, payload)
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        # Firmware caches /getCurrentInfos for ~1s; refresh after cache window.
        await asyncio.sleep(1.2)
        await self.coordinator.async_request_refresh()
