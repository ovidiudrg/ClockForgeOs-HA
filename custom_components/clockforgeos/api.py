from __future__ import annotations

from time import monotonic
from typing import Any

from aiohttp import ClientError, ClientResponseError, ClientSession


class ClockForgeOSApiError(Exception):
    """Base API error."""


class ClockForgeOSApi:
    """API client for ClockForgeOS firmware."""

    def __init__(
        self,
        session: ClientSession,
        host: str,
        password: str | None = None,
    ) -> None:
        self._session = session
        self._base = f"http://{host}"
        self._password = password
        self._token: str | None = None
        self._token_expires_at: float = 0.0
        self._last_configuration: dict[str, Any] = {}

    @property
    def has_password(self) -> bool:
        return bool(self._password)

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

    async def authenticate(self) -> None:
        """Validate password by obtaining an auth token."""
        await self._ensure_token()

    async def get_configuration(self) -> dict[str, Any]:
        """Get full configuration with passive token usage.

        This method never performs login on its own. It only uses the current
        token if one is already available, to avoid token churn on firmware
        builds that effectively allow a single active web session token.
        """
        if self._password and (not self._token or monotonic() >= self._token_expires_at):
            return self._last_configuration
        headers: dict[str, str] = {}
        if self._token:
            headers["X-Auth-Token"] = self._token

        try:
            async with self._session.get(
                f"{self._base}/getConfiguration",
                headers=headers,
                timeout=10,
            ) as response:
                if response.status == 401 and self._password:
                    # Token no longer valid. Do not re-login from polling path.
                    self._token = None
                    self._token_expires_at = 0.0
                    return self._last_configuration
                response.raise_for_status()
                payload = await response.json(content_type=None)
                if isinstance(payload, dict):
                    self._last_configuration = payload
                    return payload
                return self._last_configuration
        except (ClientError, TimeoutError, ValueError):
            # Keep integration functional even if this optional endpoint fails.
            return self._last_configuration

    async def _login(self) -> None:
        if not self._password:
            return

        try:
            async with self._session.post(
                f"{self._base}/auth/login",
                data={"password": self._password},
                timeout=10,
            ) as response:
                response.raise_for_status()
                payload = await response.json(content_type=None)
        except (ClientError, TimeoutError, ValueError) as err:
            raise ClockForgeOSApiError(f"POST /auth/login failed: {err}") from err

        token = payload.get("token")
        if not token:
            raise ClockForgeOSApiError("POST /auth/login failed: token missing in response")

        ttl_sec = int(payload.get("ttlSec", 1800))
        self._token = str(token)
        self._token_expires_at = monotonic() + max(1, ttl_sec - 5)

    async def _ensure_token(self) -> None:
        if not self._password:
            return
        if self._token and monotonic() < self._token_expires_at:
            return
        await self._login()

    async def save_setting(self, key: str, value: str) -> None:
        await self._ensure_token()

        headers: dict[str, str] = {}
        if self._token:
            headers["X-Auth-Token"] = self._token

        try:
            async with self._session.post(
                f"{self._base}/saveSetting",
                data={"key": key, "value": value},
                headers=headers,
                timeout=10,
            ) as response:
                if response.status == 401 and self._password:
                    # Token expired/invalid: refresh once and retry.
                    await self._login()
                    retry_headers = {"X-Auth-Token": self._token or ""}
                    async with self._session.post(
                        f"{self._base}/saveSetting",
                        data={"key": key, "value": value},
                        headers=retry_headers,
                        timeout=10,
                    ) as retry_response:
                        retry_response.raise_for_status()
                        return

                response.raise_for_status()
        except ClientResponseError as err:
            raise ClockForgeOSApiError(f"POST /saveSetting failed: {err.status} {err.message}") from err
        except (ClientError, TimeoutError) as err:
            raise ClockForgeOSApiError(f"POST /saveSetting failed: {err}") from err
