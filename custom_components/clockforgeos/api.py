from __future__ import annotations

from typing import Any

from aiohttp import BasicAuth, ClientError, ClientResponseError, ClientSession


class ClockForgeOSApiError(Exception):
    """Base API error."""


class ClockForgeOSApi:
    """Simple API client for ClockForgeOS firmware."""

    def __init__(self, session: ClientSession, host: str, admin_password: str | None = None) -> None:
        self._session = session
        self._base = f"http://{host}"
        self._admin_password = admin_password

    def _auth_payload(self) -> dict[str, str]:
        if not self._admin_password:
            return {}
        return {
            "password": self._admin_password,
            "admin_password": self._admin_password,
        }

    def _auth_query(self) -> dict[str, str]:
        if not self._admin_password:
            return {}
        return {
            "password": self._admin_password,
            "admin_password": self._admin_password,
        }

    def _basic_auth(self) -> BasicAuth | None:
        if not self._admin_password:
            return None
        return BasicAuth("admin", self._admin_password)

    def _auth_headers(self) -> dict[str, str]:
        if not self._admin_password:
            return {}
        return {
            "X-Admin-Password": self._admin_password,
        }

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
        data_with_password = {"key": key, "value": value, **self._auth_payload()}
        data_plain = {"key": key, "value": value}
        auth_query = self._auth_query()
        auth_headers = self._auth_headers()
        basic_auth = self._basic_auth()

        attempts: list[dict[str, Any]] = [
            {
                "data": data_with_password,
                "params": auth_query,
                "headers": auth_headers,
                "auth": basic_auth,
            },
            {
                "data": data_with_password,
                "params": auth_query,
                "headers": auth_headers,
            },
            {
                "data": data_plain,
                "headers": auth_headers,
                "auth": basic_auth,
            },
            {
                "data": data_plain,
                "params": auth_query,
            },
        ]

        last_error: Exception | None = None
        for kwargs in attempts:
            try:
                async with self._session.post(
                    f"{self._base}/saveSetting",
                    timeout=10,
                    **kwargs,
                ) as response:
                    response.raise_for_status()
                    return
            except ClientResponseError as err:
                last_error = err
                if err.status != 401:
                    raise ClockForgeOSApiError(f"POST /saveSetting failed: {err}") from err
                continue
            except (ClientError, TimeoutError) as err:
                raise ClockForgeOSApiError(f"POST /saveSetting failed: {err}") from err

        if last_error is not None:
            raise ClockForgeOSApiError(f"POST /saveSetting failed: {last_error}") from last_error

        raise ClockForgeOSApiError("POST /saveSetting failed: unknown authentication error")
