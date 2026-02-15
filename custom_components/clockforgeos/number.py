from __future__ import annotations

from homeassistant.components.number import NumberEntity, NumberMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .api import ClockForgeOSApiError
from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity

# Numeric keys accepted by firmware /saveSetting.
NUMERIC_KEYS = [
    "dayBright",
    "nightBright",
    "radarTimeout",
    "maxLedmA",
    "rgbBrightness",
    "rgbFixR",
    "rgbFixG",
    "rgbFixB",
    "rgbSpeed",
    "rgbAnimationSpeed",
    "rgbMinBrightness",
    "utc_offset",
    "tubesWakeSeconds",
    "interval",
    "alarmTimeHours",
    "alarmTimeMinutes",
    "alarmPeriod",
    "dayTimeHours",
    "dayTimeMinutes",
    "nightTimeHours",
    "nightTimeMinutes",
    "dateRepeatMin",
    "tempRepeatMin",
    "dateStart",
    "dateEnd",
    "tempStart",
    "tempEnd",
    "humidStart",
    "humidEnd",
    "pressureStart",
    "pressureEnd",
    "corrT0",
    "corrT1",
    "corrH0",
    "corrH1",
    "cathProtMin",
]

NUMERIC_META = {
    "dayBright": dict(min_value=0, max_value=255, step=1),
    "nightBright": dict(min_value=0, max_value=255, step=1),
    "radarTimeout": dict(min_value=0, max_value=600, step=1),
    "maxLedmA": dict(min_value=0, max_value=2000, step=1),
    "rgbBrightness": dict(min_value=0, max_value=255, step=1),
    "rgbFixR": dict(min_value=0, max_value=255, step=1),
    "rgbFixG": dict(min_value=0, max_value=255, step=1),
    "rgbFixB": dict(min_value=0, max_value=255, step=1),
    "rgbSpeed": dict(min_value=0, max_value=255, step=1),
    "rgbAnimationSpeed": dict(min_value=0, max_value=255, step=1),
    "rgbMinBrightness": dict(min_value=0, max_value=255, step=1),
    "utc_offset": dict(min_value=-720, max_value=720, step=1),
    "tubesWakeSeconds": dict(min_value=0, max_value=3600, step=1),
    "interval": dict(min_value=1, max_value=3600, step=1),
    "alarmTimeHours": dict(min_value=0, max_value=23, step=1),
    "alarmTimeMinutes": dict(min_value=0, max_value=59, step=1),
    "dayTimeHours": dict(min_value=0, max_value=23, step=1),
    "dayTimeMinutes": dict(min_value=0, max_value=59, step=1),
    "nightTimeHours": dict(min_value=0, max_value=23, step=1),
    "nightTimeMinutes": dict(min_value=0, max_value=59, step=1),
    "dateRepeatMin": dict(min_value=0, max_value=10, step=1),
    "tempRepeatMin": dict(min_value=0, max_value=10, step=1),
    "dateStart": dict(min_value=0, max_value=59, step=1),
    "dateEnd": dict(min_value=0, max_value=59, step=1),
    "tempStart": dict(min_value=0, max_value=59, step=1),
    "tempEnd": dict(min_value=0, max_value=59, step=1),
    "humidStart": dict(min_value=0, max_value=59, step=1),
    "humidEnd": dict(min_value=0, max_value=59, step=1),
    "pressureStart": dict(min_value=0, max_value=59, step=1),
    "pressureEnd": dict(min_value=0, max_value=59, step=1),
    "corrT0": dict(min_value=-20, max_value=20, step=0.1),
    "corrT1": dict(min_value=-20, max_value=20, step=0.1),
    "corrH0": dict(min_value=-50, max_value=50, step=0.1),
    "corrH1": dict(min_value=-50, max_value=50, step=0.1),
    "cathProtMin": dict(min_value=1, max_value=120, step=1),
}


def get_numeric_meta(key: str) -> dict:
    return NUMERIC_META.get(key, dict(min_value=0, max_value=255, step=1))


def _prettify_name(key: str) -> str:
    import re

    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1 \2", key)
    s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", s1)
    return s2.replace("_", " ").title()


def _get_time_part(value: str | None, part: str) -> float | None:
    if not value or ":" not in value:
        return None
    try:
        hh, mm = value.split(":", 1)
        return float(int(hh if part == "h" else mm))
    except (TypeError, ValueError):
        return None


def _read_value_for_key(key: str, current_info: dict, system_info: dict) -> float | None:
    # Firmware exposes HH:MM strings; map them to numeric entities.
    if key == "alarmTimeHours":
        return _get_time_part(current_info.get("alarmTime", system_info.get("alarmTime")), "h")
    if key == "alarmTimeMinutes":
        return _get_time_part(current_info.get("alarmTime", system_info.get("alarmTime")), "m")
    if key == "dayTimeHours":
        return _get_time_part(current_info.get("dayTime", system_info.get("dayTime")), "h")
    if key == "dayTimeMinutes":
        return _get_time_part(current_info.get("dayTime", system_info.get("dayTime")), "m")
    if key == "nightTimeHours":
        return _get_time_part(current_info.get("nightTime", system_info.get("nightTime")), "h")
    if key == "nightTimeMinutes":
        return _get_time_part(current_info.get("nightTime", system_info.get("nightTime")), "m")

    # Alias: firmware may report rgbSpeed while UI key is rgbAnimationSpeed.
    if key == "rgbAnimationSpeed":
        raw = current_info.get("rgbAnimationSpeed", current_info.get("rgbSpeed", system_info.get("rgbAnimationSpeed", system_info.get("rgbSpeed"))))
    else:
        raw = current_info.get(key, system_info.get(key))

    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _is_key_available(key: str, current_info: dict, system_info: dict) -> bool:
    return _read_value_for_key(key, current_info, system_info) is not None


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    current_info = coordinator.data.get("current_info", {})
    system_info = coordinator.data.get("system_info", {})

    numbers = [
        ClockForgeOSSettingNumber(coordinator, entry, key)
        for key in NUMERIC_KEYS
        if _is_key_available(key, current_info, system_info)
    ]
    async_add_entities(numbers)


class ClockForgeOSSettingNumber(ClockForgeOSEntity, NumberEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry, key: str) -> None:
        super().__init__(coordinator)
        self._key = key
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = _prettify_name(key)
        meta = get_numeric_meta(key)
        self._attr_native_min_value = meta["min_value"]
        self._attr_native_max_value = meta["max_value"]
        self._attr_native_step = meta["step"]
        self._attr_mode = NumberMode.AUTO

    @property
    def native_value(self) -> float | None:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        return _read_value_for_key(self._key, current_info, system_info)

    async def async_set_native_value(self, value: float) -> None:
        try:
            # Keep decimal precision for correction values; cast others to int.
            if self._key in {"corrT0", "corrT1", "corrH0", "corrH1"}:
                payload = f"{value:.1f}"
            else:
                payload = str(int(value))
            await self.coordinator.api.save_setting(self._key, payload)
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        await self.coordinator.async_request_refresh()
