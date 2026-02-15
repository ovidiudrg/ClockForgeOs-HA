from __future__ import annotations

from homeassistant.helpers import device_registry as dr
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .coordinator import ClockForgeOSCoordinator


class ClockForgeOSEntity(CoordinatorEntity[ClockForgeOSCoordinator]):
    """Base entity for ClockForgeOS integration."""

    _attr_has_entity_name = True

    @property
    def device_info(self) -> DeviceInfo:
        system = self.coordinator.data.get("system_info", {})
        mac = system.get("macAddress", self.coordinator.host)
        model = system.get("chipModel", "ClockForgeOS Device")
        sw = system.get("osVersion") or system.get("firmwareID") or "unknown"

        return DeviceInfo(
            identifiers={("clockforgeos", self.coordinator.host)},
            name=f"ClockForgeOS {self.coordinator.host}",
            manufacturer="ClockForgeOS",
            model=model,
            sw_version=sw,
            configuration_url=f"http://{self.coordinator.host}",
            connections={(dr.CONNECTION_NETWORK_MAC, mac)} if isinstance(mac, str) and ":" in mac else None,
        )
