from __future__ import annotations


from homeassistant.components.sensor import SensorEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DATA_COORDINATOR, DOMAIN
from .entity import ClockForgeOSEntity

SENSOR_EXCLUDE_KEYS = set([
    # Exclude keys that are already covered by other entities or are not useful as sensors
    "displayPower", "onboardLed", "enableBlink", "enableDST", "enableAutoShutoff", "tubesSleep", "wakeOnMotionEnabled", "debugEnabled", "manualDisplayOff", "alarmEnable", "mqttEnable", "enableTimeDisplay", "enableTempDisplay", "enableHumidDisplay", "enablePressDisplay", "enableDoubleBlink", "enableRadar", "cathodeProtect",
    # User-requested removals from sensors
    "dayNight",
    "isNight",
    "eepromSize",
    "eepromUsed",
    "hv5122Pins",
    "largestFreeBlock",
    "totalBytes",
    "usedBytes",
    "wifiSwitchResult",
    "wifiSwitchRollbackRunning",
    "wifiSwitchRollbackSSID",
    "wifiSwitchTargetSSID",
    # Common firmware spelling variants/typos
    "wifiSwithcResult",
    "wifiSwtichTargetSSID",
])

OPTIONAL_SECONDARY_SENSOR_KEYS = {
    "temperature2",
    "humidity2",
    "pressure2",
}

SENSOR_ICONS = {
    "temperature": "mdi:thermometer",
    "temperature1": "mdi:thermometer",
    "temperature2": "mdi:thermometer",
    "humidity": "mdi:water-percent",
    "humidity1": "mdi:water-percent",
    "humidity2": "mdi:water-percent",
    "pressure": "mdi:gauge",
    "lux": "mdi:weather-sunny",
    "lx": "mdi:weather-sunny",
    "cpuTemp": "mdi:cpu-64-bit",
    "cpuFreqMHz": "mdi:speedometer",
    "cpuCores": "mdi:chip",
    "freeHeap": "mdi:memory",
    "totalHeap": "mdi:memory",
    "largestFreeBlock": "mdi:memory",
    "uptime": "mdi:timer-outline",
    "uptimeMinutes": "mdi:timer-outline",
    "wifiSignal": "mdi:wifi",
    "wifiStatus": "mdi:wifi",
    "wifiIP": "mdi:ip-network",
    "timeSource": "mdi:clock-outline",
    "wakeSecondsLeft": "mdi:run-fast",
    "tubesPower": "mdi:lightbulb-on",
    "rssi": "mdi:wifi",
    "mqttStatus": "mdi:lan",
}


def _is_invalid_secondary_value(key: str, value: object) -> bool:
    if key not in OPTIONAL_SECONDARY_SENSOR_KEYS:
        return False
    try:
        return float(value) == 255.0
    except (TypeError, ValueError):
        return False


def _read_sensor_value(key: str, current_info: dict, config: dict, system_info: dict):
    for data in (current_info, config, system_info):
        if key in data:
            value = data[key]
            if _is_invalid_secondary_value(key, value):
                continue
            return value
    return None


def _prettify_name(key: str) -> str:
    import re
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
    return s2.replace('_', ' ').title()

async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    # Gather all keys from system_info, current_info, and config
    system_info = coordinator.data.get("system_info", {})
    current_info = coordinator.data.get("current_info", {})
    config = coordinator.data.get("config", {})
    all_keys = set(system_info) | set(current_info) | set(config)
    # Exclude keys that are handled by other platforms
    sensor_keys = sorted(
        k
        for k in all_keys
        if k not in SENSOR_EXCLUDE_KEYS
        and (
            k not in OPTIONAL_SECONDARY_SENSOR_KEYS
            or _read_sensor_value(k, current_info, config, system_info) is not None
        )
    )
    sensors = [
        ClockForgeOSDynamicSensor(coordinator, entry, key)
        for key in sensor_keys
    ]
    async_add_entities(sensors)

class ClockForgeOSDynamicSensor(ClockForgeOSEntity, SensorEntity):
    def __init__(self, coordinator, entry: ConfigEntry, key: str) -> None:
        super().__init__(coordinator)
        self._key = key
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = _prettify_name(key)
        self._attr_icon = SENSOR_ICONS.get(key)

    @property
    def native_value(self):
        # Prefer current_info, then config, then system_info.
        current_info = self.coordinator.data.get("current_info", {})
        config = self.coordinator.data.get("config", {})
        system_info = self.coordinator.data.get("system_info", {})
        return _read_sensor_value(self._key, current_info, config, system_info)
