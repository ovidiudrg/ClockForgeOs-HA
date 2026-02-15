from __future__ import annotations

from typing import Any

from aiohttp import ClientError, ClientSession


class ClockForgeOSApiError(Exception):
    """Base API error."""


class ClockForgeOSApi:
    """Simple API client for ClockForgeOS firmware."""

    def __init__(self, session: ClientSession, host: str) -> None:
        self._session = session
        self._base = f"http://{host}"

    async def _get_json(self, path: str) -> dict[str, Any]:
        try:
            async with self._session.get(f"{self._base}{path}", timeout=10) as response:
                response.raise_for_status()
                return await response.json(content_type=None)
        except (ClientError, TimeoutError, ValueError) as err:
            raise ClockForgeOSApiError(f"GET {path} failed: {err}") from err

    async def get_system_info(self) -> dict[str, Any]:
        return await self._get_json("/getSystemInfo")

    async def get_current_info(self) -> dict[str, Any]:
        return await self._get_json("/getCurrentInfos")

    async def save_setting(self, key: str, value: str) -> None:
        try:
            async with self._session.post(
                f"{self._base}/saveSetting",
                data={"key": key, "value": value},
                timeout=10,
            ) as response:
                response.raise_for_status()
        except (ClientError, TimeoutError) as err:
            raise ClockForgeOSApiError(f"POST /saveSetting failed: {err}") from err
