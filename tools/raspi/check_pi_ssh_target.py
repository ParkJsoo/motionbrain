#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ipaddress
import shutil
import socket
import subprocess
import sys


DEFAULT_CANDIDATES = ("motionbrain-pi.local", "motionbrain-pi.davolink")


def run_command(args: list[str], timeout: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
    )


def parse_ssh_config(output: str) -> dict[str, str]:
    config: dict[str, str] = {}
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or " " not in stripped:
            continue
        key, value = stripped.split(None, 1)
        config[key.lower()] = value.strip()
    return config


def ssh_config(alias: str, timeout: float) -> dict[str, str]:
    result = run_command(["ssh", "-G", alias], timeout)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"ssh -G failed for {alias}")
    return parse_ssh_config(result.stdout)


def is_ip_literal(value: str) -> bool:
    try:
        ipaddress.ip_address(value)
    except ValueError:
        return False
    return True


def resolve_host(host: str, port: int) -> list[str]:
    addresses: list[str] = []
    try:
        infos = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
    except OSError:
        return addresses
    for info in infos:
        address = info[4][0]
        if address not in addresses:
            addresses.append(address)
    return addresses


def tcp_connect(host: str, port: int, timeout: float) -> bool:
    if shutil.which("nc"):
        result = run_command(
            ["nc", "-z", "-w", str(max(1, int(timeout))), host, str(port)],
            timeout + 2,
        )
        return result.returncode == 0

    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def remote_ssh_check(alias: str, timeout: float) -> subprocess.CompletedProcess[str]:
    return run_command(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            f"ConnectTimeout={max(1, int(timeout))}",
            alias,
            "hostname; hostname -I; "
            "systemctl is-active ssh || true; "
            "systemctl is-enabled ssh.socket || true; "
            "systemctl is-active ssh.socket || true",
        ],
        timeout + 8,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Diagnose the MotionBrain Raspberry Pi SSH alias and current port-22 reachability."
    )
    parser.add_argument("alias", nargs="?", default="motionbrain-pi")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--candidate", action="append", default=[])
    parser.add_argument("--allow-static-ip", action="store_true")
    parser.add_argument("--skip-remote", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates = args.candidate or list(DEFAULT_CANDIDATES)

    try:
        config = ssh_config(args.alias, args.timeout)
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"FAIL ssh config unavailable for {args.alias}: {exc}", file=sys.stderr)
        return 1

    hostname = config.get("hostname", args.alias)
    user = config.get("user", "")
    hostkey_alias = config.get("hostkeyalias", "")
    print(f"alias={args.alias}")
    print(f"hostname={hostname}")
    if user:
        print(f"user={user}")
    if hostkey_alias:
        print(f"hostkeyalias={hostkey_alias}")

    status = 0
    if is_ip_literal(hostname) and not args.allow_static_ip:
        print(
            "FAIL HostName is a literal IP. Use motionbrain-pi.davolink or motionbrain-pi.local "
            "so DHCP address changes do not break the alias.",
            file=sys.stderr,
        )
        status = 2

    hosts_to_check = [hostname]
    for candidate in candidates:
        if candidate not in hosts_to_check:
            hosts_to_check.append(candidate)

    configured_tcp_reachable = False
    for host in hosts_to_check:
        addresses = resolve_host(host, args.port)
        address_text = ",".join(addresses) if addresses else "unresolved"
        connected = tcp_connect(host, args.port, args.timeout) if addresses else False
        print(f"tcp {host}:{args.port} addresses={address_text} reachable={str(connected).lower()}")
        configured_tcp_reachable = configured_tcp_reachable or (host == hostname and connected)

    if not configured_tcp_reachable and args.skip_remote:
        print(f"FAIL configured HostName is not reachable on TCP/{args.port}: {hostname}", file=sys.stderr)
        status = 1 if status == 0 else status

    if not args.skip_remote:
        try:
            remote = remote_ssh_check(args.alias, args.timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"FAIL remote ssh check failed: {exc}", file=sys.stderr)
            return 1 if status == 0 else status
        if remote.returncode != 0:
            if not configured_tcp_reachable:
                print(f"FAIL configured HostName is not reachable on TCP/{args.port}: {hostname}", file=sys.stderr)
            print(remote.stderr.strip() or remote.stdout.strip(), file=sys.stderr)
            return 1 if status == 0 else status
        print("remote_check=ok")
        print(remote.stdout.strip())

    return status


if __name__ == "__main__":
    raise SystemExit(main())
