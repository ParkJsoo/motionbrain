from __future__ import annotations

import importlib.util
import ipaddress
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "tools" / "raspi" / "discover_device_url.py"
SPEC = importlib.util.spec_from_file_location("discover_device_url", SCRIPT_PATH)
assert SPEC is not None
discover_device_url = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(discover_device_url)


class RaspiDiscoveryTest(unittest.TestCase):
    def test_normalize_base_url_strips_path(self) -> None:
        self.assertEqual(
            discover_device_url.normalize_base_url("motionbrain-cam.local/status"),
            "http://motionbrain-cam.local",
        )
        self.assertEqual(
            discover_device_url.normalize_base_url("http://192.168.1.22/status?x=1"),
            "http://192.168.1.22",
        )

    def test_matches_camera_status(self) -> None:
        self.assertTrue(discover_device_url.matches_kind({"node": "esp32cam"}, "camera"))
        self.assertTrue(discover_device_url.matches_kind({"hostname": "motionbrain-cam"}, "camera"))
        self.assertTrue(discover_device_url.matches_kind({"frameSize": "qvga", "psram": True}, "camera"))
        self.assertFalse(discover_device_url.matches_kind({"messageType": "status"}, "camera"))

    def test_matches_controller_status(self) -> None:
        payload = {"messageType": "status", "state": "IDLE", "motors": {}}
        self.assertTrue(discover_device_url.matches_kind(payload, "controller"))
        self.assertTrue(discover_device_url.matches_kind({"schemaVersion": "phase3.v1"}, "controller"))
        self.assertFalse(discover_device_url.matches_kind({"node": "esp32cam"}, "controller"))

    def test_prefixes_from_ip_addr_limits_large_network_to_local_24(self) -> None:
        output = "2 wlan0 inet 192.168.219.110/16 brd 192.168.255.255 scope global wlan0"
        networks = discover_device_url.prefixes_from_ip_addr(output, max_hosts=512)
        self.assertEqual(networks, [ipaddress.ip_network("192.168.219.0/24")])

    def test_candidate_urls_uses_host_addresses(self) -> None:
        urls = discover_device_url.candidate_urls([ipaddress.ip_network("192.168.7.0/30")])
        self.assertEqual(urls, ["http://192.168.7.1", "http://192.168.7.2"])


if __name__ == "__main__":
    unittest.main()
