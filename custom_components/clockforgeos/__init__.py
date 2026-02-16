from __future__ import annotations

from datetime import timedelta

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ConfigEntryNotReady
from homeassistant.helpers import entity_registry as er

from .const import (
    CONF_SCAN_INTERVAL,
    DATA_COORDINATOR,
    DEFAULT_SCAN_INTERVAL,
    DOMAIN,
    PLATFORMS,
)
from .coordinator import ClockForgeOSCoordinator

REMOVED_SENSOR_KEYS = {
    "dayNight",
    "isNight",
    "dayNightIsNight",
    "eepromSize",
    "eepromUsed",
    "hv5122Pins",
    "largestFreeBlock",
    "totalBytes",
    "usedBytes",
    "wifiSwitchResult",
    "wifiSwitchRollbackRunning",
    "wifiSwitchRollbackSSID",
    "wifiSwitchRollbackSsid",
    "wifiSwitchTargetSSID",
    "wifiSwitchTargetSsid",
    "wifiSwithcResult",
    "wifiSwtichTargetSSID",
}


def _unique_id_key(entry: ConfigEntry, entity: er.RegistryEntry) -> str | None:
    if not entity.unique_id:
        return None
    prefix = f"{entry.entry_id}_"
    if not entity.unique_id.startswith(prefix):
        return None
    return entity.unique_id[len(prefix):]


def _is_legacy_alarm_minute_entity(entity: er.RegistryEntry) -> bool:
    if entity.platform != DOMAIN:
        return False
    if entity.domain == "select" and entity.unique_id.endswith("_alarmMinute"):
        return True
    return False


def _is_removed_sensor_entity(entry: ConfigEntry, entity: er.RegistryEntry) -> bool:
    if entity.platform != DOMAIN or entity.domain != "sensor":
        return False
    key = _unique_id_key(entry, entity)
    if key is None:
        return False
    return key in REMOVED_SENSOR_KEYS


async def _async_cleanup_legacy_entities(hass: HomeAssistant, entry: ConfigEntry) -> None:
    registry = er.async_get(hass)
    entries = er.async_entries_for_config_entry(registry, entry.entry_id)
    for entity in entries:
        if _is_legacy_alarm_minute_entity(entity) or _is_removed_sensor_entity(entry, entity):
            registry.async_remove(entity.entity_id)


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up ClockForgeOS from a config entry."""
    host = entry.data[CONF_HOST]
    scan_interval = entry.options.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL)
    password = (entry.options.get("password") or "").strip() or None

    coordinator = ClockForgeOSCoordinator(
        hass=hass,
        host=host,
        password=password,
        update_interval=timedelta(seconds=scan_interval),
    )

    try:
        await coordinator.async_config_entry_first_refresh()
    except Exception as err:
        raise ConfigEntryNotReady(f"Unable to connect to ClockForgeOS at {host}: {err}") from err

    await _async_cleanup_legacy_entities(hass, entry)
    hass.data.setdefault(DOMAIN, {})[entry.entry_id] = {DATA_COORDINATOR: coordinator}
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if unload_ok:
        hass.data[DOMAIN].pop(entry.entry_id, None)
    return unload_ok


async def async_reload_entry(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Reload a config entry."""
    await async_unload_entry(hass, entry)
    await async_setup_entry(hass, entry)
