from __future__ import annotations

from homeassistant.components.button import ButtonEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.exceptions import HomeAssistantError

from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity
from .api import ClockForgeOSApiError

# List of all action endpoints to expose as buttons
BUTTON_ACTIONS = [
    ("reset", "Reset Device", "mdi:restart"),
    ("factoryreset", "Factory Reset", "mdi:factory"),
    ("firmwareupdate", "Firmware Update", "mdi:update"),
    ("cathodeProtect", "Cathode Protect", "mdi:shield"),
    ("scanWifi", "Scan WiFi", "mdi:wifi"),
    ("connectWifi", "Connect WiFi", "mdi:wifi-arrow-right"),
    ("setManualTime", "Set Manual Time", "mdi:clock-edit"),
]

async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator = hass.data[DOMAIN][entry.entry_id][DATA_COORDINATOR]
    buttons = [
        ClockForgeOSActionButton(coordinator, entry, endpoint, name, icon)
        for endpoint, name, icon in BUTTON_ACTIONS
    ]
    async_add_entities(buttons)

class ClockForgeOSActionButton(ClockForgeOSEntity, ButtonEntity):
    def __init__(self, coordinator: ClockForgeOSCoordinator, entry: ConfigEntry, endpoint: str, name: str, icon: str) -> None:
        super().__init__(coordinator)
        self._endpoint = endpoint
        self._attr_unique_id = f"{entry.entry_id}_{endpoint}"
        self._attr_name = name
        self._attr_icon = icon

    async def async_press(self) -> None:
        try:
            await self._call_action()
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to call {self._endpoint}: {err}") from err
        await self.coordinator.async_request_refresh()

    async def _call_action(self) -> None:
        # POST to /<endpoint> (no payload for most actions)
        await self.coordinator.api._session.post(f"{self.coordinator.api._base}/{self._endpoint}", timeout=10)
