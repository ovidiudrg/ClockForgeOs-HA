from __future__ import annotations

from homeassistant.components.select import SelectEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.exceptions import HomeAssistantError

from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity
from .api import ClockForgeOSApiError

# List of all enumerated /saveSetting keys to expose as selects
SELECT_KEYS = {
    "rgbEffect": [
        "off", "static", "rainbow", "breath", "chase", "custom"
    ],
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
        return str(value)

    async def async_select_option(self, option: str) -> None:
        try:
            await self.coordinator.api.save_setting(self._key, option)
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        await self.coordinator.async_request_refresh()
