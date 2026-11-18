"""Build one base nudge bound to the generation observed before sending it."""
import re
from typing import Any
from urllib.parse import urlencode


def base_start_path(snapshot: dict[str, Any], direction: str, percent: int) -> str:
    if direction not in ("left", "right") or type(percent) is not int or not 1 <= percent <= 100:
        raise ValueError("invalid base nudge")
    if not isinstance(snapshot, dict) or snapshot.get("schemaVersion") != "motionbrain.manual_control_leases.v1":
        raise ValueError("controller does not support versioned manual leases")
    boot = snapshot.get("bootId")
    motors = snapshot.get("motors")
    if not isinstance(boot, str) or not re.fullmatch(r"[0-9a-f]{32}", boot) or not isinstance(motors, list):
        raise ValueError("invalid manual lease snapshot")
    base = [m for m in motors if isinstance(m, dict) and m.get("motor") == 5]
    if len(base) != 1 or base[0].get("active") is not False:
        raise ValueError("base lease unavailable")
    generation = base[0].get("generation")
    if type(generation) is not int or not 1 <= generation < 2**32:
        raise ValueError("invalid base lease generation")
    params = {
        "joint": "base",
        "action": direction,
        "percent": percent,
        "lease": "start",
        "expectedBootId": boot,
        "expectedGeneration": generation,
    }
    return "/joint?" + urlencode(params)
