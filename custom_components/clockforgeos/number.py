from __future__ import annotations

from homeassistant.components.number import NumberEntity, NumberMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.exceptions import HomeAssistantError

from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity
from .api import ClockForgeOSApiError

# List of all numeric /saveSetting keys to expose as numbers (add more as needed)
NUMERIC_KEYS = [
    "dayBright", "nightBright", "radarTimeout", "maxLedmA", "rgbBrightness", "rgbFixR", "rgbFixG", "rgbFixB", "rgbSpeed", "rgbMinBrightness",
    "utc_offset", "tubesWakeSeconds", "interval", "alarmTimeHours", "alarmTimeMinutes", "alarmPeriod",
    "dayTimeHours", "dayTimeMinutes", "nightTimeHours", "nightTimeMinutes",
    "dateRepeatMin", "tempRepeatMin", "dateStart", "dateEnd", "tempStart", "tempEnd", "humidStart", "humidEnd", "pressureStart", "pressureEnd",
    "corrT0", "corrT1", "corrH0", "corrH1", "cathProtMin", "uiWidth"
]

# Optionally, define min/max/step for each key
NUMERIC_META = {
    "dayBright": dict(min_value=0, max_value=255, step=1),
    "nightBright": dict(min_value=0, max_value=255, step=1),
    "radarTimeout": dict(min_value=0, max_value=600, step=1),
    "maxLedmA": dict(min_value=0, max_value=1000, step=1),
    "rgbBrightness": dict(min_value=0, max_value=255, step=1),
    "rgbFixR": dict(min_value=0, max_value=255, step=1),
    "rgbFixG": dict(min_value=0, max_value=255, step=1),
    "rgbFixB": dict(min_value=0, max_value=255, step=1),
    "rgbSpeed": dict(min_value=0, max_value=255, step=1),
    "rgbMinBrightness": dict(min_value=0, max_value=255, step=1),
    "utc_offset": dict(min_value=-720, max_value=720, step=1),
    "tubesWakeSeconds": dict(min_value=0, max_value=3600, step=1),
    "interval": dict(min_value=1, max_value=3600, step=1),
    # ...add more as needed
}

def get_numeric_meta(key):
    return NUMERIC_META.get(key, dict(min_value=0, max_value=255, step=1))

async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    numbers = [
        ClockForgeOSSettingNumber(coordinator, entry, key)
        for key in NUMERIC_KEYS
    ]
    async_add_entities(numbers)

class ClockForgeOSSettingNumber(ClockForgeOSEntity, NumberEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry, key: str) -> None:
        super().__init__(coordinator)
        self._key = key
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = self._prettify_name(key)
        meta = get_numeric_meta(key)
        self._attr_native_min_value = meta["min_value"]
        self._attr_native_max_value = meta["max_value"]
        self._attr_native_step = meta["step"]
        self._attr_mode = NumberMode.AUTO

    @staticmethod
    def _prettify_name(key: str) -> str:
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
        s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
        return s2.replace('_', ' ').title()

    @property
    def native_value(self) -> float | None:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        value = current_info.get(self._key, system_info.get(self._key))
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    async def async_set_native_value(self, value: float) -> None:
        try:
            await self.coordinator.api.save_setting(self._key, str(int(value)))
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        await self.coordinator.async_request_refresh()
