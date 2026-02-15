from __future__ import annotations

from homeassistant.components.switch import SwitchEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .api import ClockForgeOSApiError
from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    async_add_entities([ClockForgeOSDisplayPowerSwitch(coordinator, entry)])


class ClockForgeOSDisplayPowerSwitch(ClockForgeOSEntity, SwitchEntity):
    _attr_name = "Display Power"
    _attr_icon = "mdi:monitor"

    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry) -> None:
        super().__init__(coordinator)
        self._attr_unique_id = f"{entry.entry_id}_display_power"

    @property
    def is_on(self) -> bool:
        current_info = self.coordinator.data.get("current_info", {})
        system_info = self.coordinator.data.get("system_info", {})
        value = current_info.get("displayPower", system_info.get("displayPower", 1))
        return str(value) not in ("0", "false", "False")

    async def async_turn_on(self, **kwargs) -> None:
        await self._async_set_display_power(True)

    async def async_turn_off(self, **kwargs) -> None:
        await self._async_set_display_power(False)

    async def _async_set_display_power(self, enabled: bool) -> None:
        try:
            await self.coordinator.api.save_setting("displayPower", "1" if enabled else "0")
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set displayPower: {err}") from err
        await self.coordinator.async_request_refresh()
