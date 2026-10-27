#!/usr/bin/env python3
"""Publish a short-lived dashboard rendezvous; no image or motion requests."""
from __future__ import annotations

import json
import os
import sys
import urllib.parse
import urllib.request


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def publish(controller_url: str, dashboard_url: str, token: str, *, opener=None) -> bool:
    if not token or token.startswith("CHANGE_ME"):
        return False
    controller = urllib.parse.urlsplit(controller_url)
    dashboard = urllib.parse.urlsplit(dashboard_url)
    if (controller.scheme != "http" or not controller.hostname or controller.username or
            controller.password or controller.query or controller.fragment or controller.path not in ("", "/")):
        return False
    port = dashboard.port or 80
    if dashboard.scheme != "http" or not 1024 <= port <= 65535:
        return False
    opener = opener or urllib.request.build_opener(NoRedirect)
    # Only announce a running compatible dashboard, never a stopped service.
    with opener.open(dashboard_url.rstrip("/") + "/api/config", timeout=2.0) as response:
        config = json.loads(response.read(16384))
    if config.get("service") != "motionbrain-dashboard" or config.get("recoveryVersion") != 1:
        return False
    request = urllib.request.Request(
        controller_url.rstrip("/") + f"/vision_endpoint?port={port}", data=b"", method="POST",
        headers={"X-MotionBrain": "1", "X-MotionBrain-Token": token},
    )
    with opener.open(request, timeout=2.0) as response:
        result = json.loads(response.read(4096))
    return result.get("schema") == "motionbrain.vision_endpoint.v1" and result.get("available") is True


def main() -> int:
    try:
        ok = publish(sys.argv[1], sys.argv[2], os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""))
    except Exception:
        # Do not put credential-bearing requests or response bodies into journalctl.
        ok = False
    if not ok:
        print("Dashboard address publication unavailable; check compatible firmware and controller token", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
