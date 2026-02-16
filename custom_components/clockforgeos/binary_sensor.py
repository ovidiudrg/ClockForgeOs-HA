from __future__ import annotations

from homeassistant.components.binary_sensor import BinarySensorEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DATA_COORDINATOR, DOMAIN
from .entity import ClockForgeOSEntity


def _parse_bool(value: object) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    s = str(value).strip().lower()
    if s in {"1", "true", "on", "yes"}:
        return True
    if s in {"0", "false", "off", "no"}:
        return False
    return None


class ClockForgeOSRadarMotionBinarySensor(ClockForgeOSEntity, BinarySensorEntity):
    _attr_name = "Radar Motion"
    _attr_icon = "mdi:radar"

    def __init__(self, coordinator, entry: ConfigEntry) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = f"{entry.entry_id}_radar_motion"

    @property
    def available(self) -> bool:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        config = self.coordinator.data.get("config", {})

        if any(k in current_info or k in config or k in system_info for k in ("radarMotion", "motion", "wakeSecondsLeft")):
            return True
        return False

    @property
    def is_on(self) -> bool | None:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        config = self.coordinator.data.get("config", {})

        # Prefer explicit motion keys if present.
        for key in ("radarMotion", "motion"):
            if key in current_info:
                parsed = _parse_bool(current_info.get(key))
                if parsed is not None:
                    return parsed
            if key in config:
                parsed = _parse_bool(config.get(key))
                if parsed is not None:
                    return parsed
            if key in system_info:
                parsed = _parse_bool(system_info.get(key))
                if parsed is not None:
                    return parsed

        # Fallback: wake window active implies recent motion.
        wake_left = current_info.get("wakeSecondsLeft", config.get("wakeSecondsLeft", system_info.get("wakeSecondsLeft")))
        try:
            return int(float(wake_left)) > 0
        except (TypeError, ValueError):
            return None


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    async_add_entities([ClockForgeOSRadarMotionBinarySensor(coordinator, entry)])
