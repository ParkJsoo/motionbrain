#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ipaddress
import json
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import as_completed
from typing import Any


def normalize_base_url(value: str) -> str:
    value = value.strip()
    if not value:
        return ""
    if "://" not in value:
        value = f"http://{value}"
    parsed = urllib.parse.urlparse(value)
    if not parsed.netloc:
        return value.rstrip("/")
    return urllib.parse.urlunparse((parsed.scheme or "http", parsed.netloc, "", "", "", "")).rstrip("/")


def status_url(base_url: str) -> str:
    return f"{normalize_base_url(base_url)}/status"


def fetch_status(base_url: str, timeout: float) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(status_url(base_url), timeout=timeout) as response:
            body = response.read(8192)
    except (OSError, TimeoutError, urllib.error.URLError):
        return None

    try:
        payload = json.loads(body.decode("utf-8", errors="replace"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def matches_kind(payload: dict[str, Any], kind: str) -> bool:
    if kind == "camera":
        return (
            payload.get("node") == "esp32cam"
            or payload.get("hostname") == "motionbrain-cam"
            or ("frameSize" in payload and "psram" in payload)
        )
    if kind == "controller":
        return (
            payload.get("messageType") == "status"
            and isinstance(payload.get("motors"), dict)
            and "state" in payload
        ) or str(payload.get("schemaVersion", "")).startswith("phase")
    raise ValueError(f"unsupported kind: {kind}")


def prefixes_from_ip_addr(output: str, *, max_hosts: int) -> list[ipaddress.IPv4Network]:
    networks: list[ipaddress.IPv4Network] = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            interface = ipaddress.ip_interface(parts[3])
        except ValueError:
            continue
        if interface.ip.is_loopback or interface.ip.is_link_local:
            continue

        network = interface.network
        if network.num_addresses > max_hosts:
            octets = str(interface.ip).split(".")
            network = ipaddress.ip_network(".".join(octets[:3]) + ".0/24")
        if network not in networks:
            networks.append(network)
    return networks


def local_networks(max_hosts: int) -> list[ipaddress.IPv4Network]:
    try:
        output = subprocess.check_output(
            ["ip", "-o", "-4", "addr", "show", "scope", "global"],
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    return prefixes_from_ip_addr(output, max_hosts=max_hosts)


def parse_cidr_values(values: list[str], *, max_hosts: int) -> list[ipaddress.IPv4Network]:
    networks: list[ipaddress.IPv4Network] = []
    for value in values:
        if not value:
            continue
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            network = ipaddress.ip_network(item, strict=False)
            if network.version != 4:
                continue
            if network.num_addresses > max_hosts:
                raise ValueError(f"CIDR has too many addresses for discovery: {item}")
            if network not in networks:
                networks.append(network)
    return networks


def candidate_urls(networks: list[ipaddress.IPv4Network]) -> list[str]:
    urls: list[str] = []
    for network in networks:
        for host in network.hosts():
            urls.append(f"http://{host}")
    return urls


def discover_url(
    *,
    kind: str,
    preferred: str,
    timeout: float,
    workers: int,
    networks: list[ipaddress.IPv4Network],
) -> str | None:
    preferred = normalize_base_url(preferred)
    if preferred:
        payload = fetch_status(preferred, timeout)
        if payload is not None and matches_kind(payload, kind):
            return preferred

    urls = candidate_urls(networks)
    with ThreadPoolExecutor(max_workers=max(workers, 1)) as executor:
        futures = {executor.submit(fetch_status, url, timeout): url for url in urls}
        for future in as_completed(futures):
            payload = future.result()
            if payload is not None and matches_kind(payload, kind):
                return futures[future]
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Discover MotionBrain devices by /status on the local LAN.")
    parser.add_argument("--kind", choices=("controller", "camera"), required=True)
    parser.add_argument("--preferred", default="")
    parser.add_argument("--timeout", type=float, default=0.35)
    parser.add_argument("--workers", type=int, default=64)
    parser.add_argument("--max-hosts", type=int, default=512)
    parser.add_argument("--cidr", action="append", default=[], help="Optional CIDR(s) to scan instead of local /24s")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        networks = parse_cidr_values(args.cidr, max_hosts=args.max_hosts) if args.cidr else local_networks(args.max_hosts)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    resolved = discover_url(
        kind=args.kind,
        preferred=args.preferred,
        timeout=args.timeout,
        workers=args.workers,
        networks=networks,
    )
    if not resolved:
        return 1
    print(resolved)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
