from __future__ import annotations

from typing import Any

import voluptuous as vol
from homeassistant import config_entries
from homeassistant.const import CONF_HOST
from homeassistant.helpers.aiohttp_client import async_get_clientsession
from homeassistant.helpers import selector

from .api import ClockForgeOSApi, ClockForgeOSApiError
from .const import (
    CONF_ADMIN_PASSWORD,
    CONF_SCAN_INTERVAL,
    DEFAULT_SCAN_INTERVAL,
    DOMAIN,
)


class ClockForgeOSConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Handle a config flow for ClockForgeOS."""

    VERSION = 1

    async def async_step_user(self, user_input: dict[str, Any] | None = None):
        errors: dict[str, str] = {}

        if user_input is not None:
            host = user_input[CONF_HOST].strip()
            admin_password = (user_input.get(CONF_ADMIN_PASSWORD) or "").strip()
            await self.async_set_unique_id(host)
            self._abort_if_unique_id_configured()

            api = ClockForgeOSApi(async_get_clientsession(self.hass), host, admin_password or None)
            try:
                await api.get_system_info()
            except ClockForgeOSApiError:
                errors["base"] = "cannot_connect"
            else:
                return self.async_create_entry(
                    title=f"ClockForgeOS ({host})",
                    data={CONF_HOST: host},
                    options={
                        CONF_SCAN_INTERVAL: user_input[CONF_SCAN_INTERVAL],
                        CONF_ADMIN_PASSWORD: admin_password,
                    },
                )

        schema = vol.Schema(
            {
                vol.Required(CONF_HOST): str,
                vol.Optional(CONF_SCAN_INTERVAL, default=DEFAULT_SCAN_INTERVAL): vol.All(
                    vol.Coerce(int), vol.Range(min=3, max=300)
                ),
                vol.Optional(CONF_ADMIN_PASSWORD, default=""): selector.TextSelector(
                    selector.TextSelectorConfig(type=selector.TextSelectorType.PASSWORD)
                ),
            }
        )

        return self.async_show_form(step_id="user", data_schema=schema, errors=errors)

    @staticmethod
    def async_get_options_flow(config_entry):
        return ClockForgeOSOptionsFlow(config_entry)


class ClockForgeOSOptionsFlow(config_entries.OptionsFlow):
    """Handle ClockForgeOS options."""

    def __init__(self, config_entry: config_entries.ConfigEntry) -> None:
        self.config_entry = config_entry

    async def async_step_init(self, user_input: dict[str, Any] | None = None):
        if user_input is not None:
            return self.async_create_entry(title="", data=user_input)

        schema = vol.Schema(
            {
                vol.Optional(
                    CONF_SCAN_INTERVAL,
                    default=self.config_entry.options.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL),
                ): vol.All(vol.Coerce(int), vol.Range(min=3, max=300))
                ,
                vol.Optional(
                    CONF_ADMIN_PASSWORD,
                    default=self.config_entry.options.get(CONF_ADMIN_PASSWORD, ""),
                ): selector.TextSelector(
                    selector.TextSelectorConfig(type=selector.TextSelectorType.PASSWORD)
                ),
            }
        )

        return self.async_show_form(step_id="init", data_schema=schema)
