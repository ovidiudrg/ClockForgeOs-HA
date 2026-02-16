from __future__ import annotations

import asyncio

from homeassistant.components.select import SelectEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.exceptions import HomeAssistantError

from .const import DATA_COORDINATOR, DOMAIN
from .coordinator import ClockForgeOSCoordinator
from .entity import ClockForgeOSEntity
from .api import ClockForgeOSApiError

RGB_EFFECT_LABELS = {
    "0": "OFF",
    "1": "Fixed Color",
    "2": "Rainbow Flow",
    "3": "Rainbow Stepper",
    "4": "Color Dimmer",
    "5": "Color Changer",
    "6": "Color FlowChanger",
    "7": "Color FlowChanger Random-1",
    "8": "Color FlowChanger Random-2",
    "9": "Color FlowChanger Random-3",
    "10": "Knight Rider's KITT",
    "11": "HeartBeat",
}

RGB_FIXED_PALETTE: list[tuple[str, int, int, int]] = [
    ("Red", 255, 0, 0),
    ("Orange 1", 255, 48, 0),
    ("Orange 2", 255, 96, 0),
    ("Amber 1", 255, 160, 0),
    ("Amber 2", 255, 220, 0),
    ("Lime 1", 180, 255, 0),
    ("Lime 2", 100, 255, 0),
    ("Green", 0, 255, 0),
    ("Mint", 0, 255, 120),
    ("Cyan", 0, 255, 220),
    ("Sky Blue", 0, 180, 255),
    ("Blue", 0, 110, 255),
    ("Royal Blue", 0, 40, 255),
    ("Violet", 90, 0, 255),
    ("Purple", 160, 0, 255),
    ("Magenta", 230, 0, 255),
    ("Pink 1", 255, 0, 190),
    ("Pink 2", 255, 0, 120),
    ("White", 255, 255, 255),
    ("Warm White", 255, 190, 120),
    ("Warm Amber", 255, 120, 20),
    ("Rose", 255, 80, 160),
]

# List of all enumerated /saveSetting keys to expose as selects
SELECT_KEYS = {
    "rgbEffect": list(RGB_EFFECT_LABELS.values()),
    "rgbPalette": [item[0] for item in RGB_FIXED_PALETTE],
    "alarmAmPm": ["AM", "PM"],
    "alarmMinute": [f"{i:02d}" for i in range(60)],
}

SELECT_DISPLAY_NAMES = {
    "alarmAmPm": "Alarm AM/PM",
    "alarmMinute": "Alarm Minute",
}


def _normalize_effect_key(value: object) -> str | None:
    if value is None:
        return None
    value_str = str(value).strip()
    if value_str in RGB_EFFECT_LABELS:
        return value_str
    try:
        return str(int(float(value_str)))
    except (TypeError, ValueError):
        return None


def _parse_alarm_time(value: object) -> tuple[int, int] | None:
    if value is None:
        return None
    text = str(value).strip()
    if ":" not in text:
        return None
    try:
        hh_str, mm_str = text.split(":", 1)
        hh = int(hh_str)
        mm = int(mm_str)
    except (TypeError, ValueError):
        return None
    if hh < 0 or hh > 23 or mm < 0 or mm > 59:
        return None
    return hh, mm


def _read_alarm_hour_24(current_info: dict, config: dict, system_info: dict) -> int | None:
    parsed = _parse_alarm_time(
        current_info.get("alarmTime", config.get("alarmTime", system_info.get("alarmTime")))
    )
    if parsed is None:
        return None
    return parsed[0]


def _read_alarm_minute(current_info: dict, config: dict, system_info: dict) -> int | None:
    parsed = _parse_alarm_time(
        current_info.get("alarmTime", config.get("alarmTime", system_info.get("alarmTime")))
    )
    if parsed is None:
        return None
    return parsed[1]


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
        self._label_to_value = {v: k for k, v in RGB_EFFECT_LABELS.items()} if key == "rgbEffect" else {}
        self._palette_label_to_rgb = {name: (r, g, b) for name, r, g, b in RGB_FIXED_PALETTE} if key == "rgbPalette" else {}
        self._attr_unique_id = f"{entry.entry_id}_{key}"
        self._attr_name = SELECT_DISPLAY_NAMES.get(key, self._prettify_name(key))
        self._attr_options = options

    @staticmethod
    def _prettify_name(key: str) -> str:
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', key)
        s2 = re.sub('([a-z0-9])([A-Z])', r'\1 \2', s1)
        return s2.replace('_', ' ').title()

    @property
    def available(self) -> bool:
        current_info = self.coordinator.data.get("current_info", {})
        config = self.coordinator.data.get("config", {})
        system_info = self.coordinator.data.get("system_info", {})
        if self._key == "alarmAmPm":
            return _read_alarm_hour_24(current_info, config, system_info) is not None
        if self._key == "alarmMinute":
            return _read_alarm_minute(current_info, config, system_info) is not None
        rgb_key = _normalize_effect_key(
            current_info.get("rgbEffect", config.get("rgbEffect", system_info.get("rgbEffect")))
        )
        if rgb_key is None:
            return self._key != "rgbPalette"
        if rgb_key == "255":
            return False
        return True

    @property
    def current_option(self) -> str | None:
        current_info = self.coordinator.data.get("current_info", {})
        config = self.coordinator.data.get("config", {})
        system_info = self.coordinator.data.get("system_info", {})
        if self._key == "rgbEffect":
            value = current_info.get("rgbEffect", config.get("rgbEffect", system_info.get("rgbEffect")))
            if value is None:
                return None
            value_key = _normalize_effect_key(value)
            if value_key in RGB_EFFECT_LABELS:
                return RGB_EFFECT_LABELS[value_key]
            value_str = str(value).strip()
            if value_str in self._options:
                return value_str
            return None
        if self._key == "rgbPalette":
            r = current_info.get("rgbFixR", config.get("rgbFixR", system_info.get("rgbFixR")))
            g = current_info.get("rgbFixG", config.get("rgbFixG", system_info.get("rgbFixG")))
            b = current_info.get("rgbFixB", config.get("rgbFixB", system_info.get("rgbFixB")))
            try:
                rgb = (int(float(r)), int(float(g)), int(float(b)))
            except (TypeError, ValueError):
                return None
            for name, pr, pg, pb in RGB_FIXED_PALETTE:
                if rgb == (pr, pg, pb):
                    return name
            return None
        if self._key == "alarmAmPm":
            hour_24 = _read_alarm_hour_24(current_info, config, system_info)
            if hour_24 is None:
                return None
            return "PM" if hour_24 >= 12 else "AM"
        if self._key == "alarmMinute":
            minute = _read_alarm_minute(current_info, config, system_info)
            if minute is None:
                return None
            return f"{minute:02d}"
        value = current_info.get(self._key, system_info.get(self._key))
        return str(value) if value is not None else None

    async def async_select_option(self, option: str) -> None:
        try:
            if self._key == "rgbEffect":
                payload = self._label_to_value.get(option, option)
                await self.coordinator.api.save_setting("rgbEffect", payload)
            elif self._key == "rgbPalette":
                rgb = self._palette_label_to_rgb.get(option)
                if rgb is None:
                    raise HomeAssistantError(f"Invalid palette option: {option}")
                r, g, b = rgb
                # Force fixed color mode, then apply palette RGB.
                await self.coordinator.api.save_setting("rgbEffect", "1")
                await self.coordinator.api.save_setting("rgbFixR", str(r))
                await self.coordinator.api.save_setting("rgbFixG", str(g))
                await self.coordinator.api.save_setting("rgbFixB", str(b))
            elif self._key == "alarmAmPm":
                if option not in {"AM", "PM"}:
                    raise HomeAssistantError(f"Invalid alarm meridiem option: {option}")
                current_info = self.coordinator.data.get("current_info", {})
                config = self.coordinator.data.get("config", {})
                system_info = self.coordinator.data.get("system_info", {})
                hour_24 = _read_alarm_hour_24(current_info, config, system_info)
                if hour_24 is None:
                    raise HomeAssistantError("Alarm time unavailable")
                if option == "AM" and hour_24 >= 12:
                    hour_24 -= 12
                elif option == "PM" and hour_24 < 12:
                    hour_24 += 12
                await self.coordinator.api.save_setting("alarmTimeHours", str(hour_24))
            elif self._key == "alarmMinute":
                try:
                    minute = int(option)
                except ValueError as err:
                    raise HomeAssistantError(f"Invalid alarm minute option: {option}") from err
                if minute < 0 or minute > 59:
                    raise HomeAssistantError(f"Invalid alarm minute option: {option}")
                await self.coordinator.api.save_setting("alarmTimeMinutes", str(minute))
            else:
                await self.coordinator.api.save_setting(self._key, option)
        except ClockForgeOSApiError as err:
            raise HomeAssistantError(f"Failed to set {self._key}: {err}") from err
        # Firmware caches /getCurrentInfos for ~1s; refresh after cache window.
        await asyncio.sleep(1.2)
        await self.coordinator.async_request_refresh()
