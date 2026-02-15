from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from homeassistant.components.sensor import SensorEntity, SensorEntityDescription
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EntityCategory, UnitOfTemperature
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DATA_COORDINATOR, DOMAIN
from .entity import ClockForgeOSEntity


@dataclass(frozen=True)
class ClockForgeOSSensorDescription(SensorEntityDescription):
    value_fn: Callable[[dict[str, Any], dict[str, Any]], Any] = lambda _s, _c: None


SENSORS: tuple[ClockForgeOSSensorDescription, ...] = (
    ClockForgeOSSensorDescription(
        key="os_version",
        name="OS Version",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("osVersion"),
    ),
    ClockForgeOSSensorDescription(
        key="firmware_id",
        name="Firmware ID",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("firmwareID"),
    ),
    ClockForgeOSSensorDescription(
        key="wifi_status",
        name="WiFi Status",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("wifiStatus"),
    ),
    ClockForgeOSSensorDescription(
        key="wifi_signal",
        name="WiFi Signal",
        native_unit_of_measurement="dBm",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("wifiSignal"),
    ),
    ClockForgeOSSensorDescription(
        key="cpu_temp",
        name="CPU Temperature",
        native_unit_of_measurement=UnitOfTemperature.CELSIUS,
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("cpuTemp"),
    ),
    ClockForgeOSSensorDescription(
        key="free_heap",
        name="Free Heap",
        native_unit_of_measurement="B",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("freeHeap"),
    ),
    ClockForgeOSSensorDescription(
        key="uptime_minutes",
        name="Uptime Minutes",
        native_unit_of_measurement="min",
        entity_category=EntityCategory.DIAGNOSTIC,
        value_fn=lambda s, _c: s.get("uptimeMinutes"),
    ),
    ClockForgeOSSensorDescription(
        key="current_time",
        name="Clock Time",
        icon="mdi:clock-digital",
        value_fn=lambda _s, c: c.get("currentDateTime") or c.get("currentTime"),
    ),
)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    async_add_entities(ClockForgeOSSensor(coordinator, entry, desc) for desc in SENSORS)


class ClockForgeOSSensor(ClockForgeOSEntity, SensorEntity):
    def __init__(self, coordinator, entry: ConfigEntry, description: ClockForgeOSSensorDescription) -> None:
        super().__init__(coordinator)
        self.entity_description = description
        self._attr_unique_id = f"{entry.entry_id}_{description.key}"

    @property
    def native_value(self):
        system_info = self.coordinator.data.get("system_info", {})
        current_info = self.coordinator.data.get("current_info", {})
        return self.entity_description.value_fn(system_info, current_info)
