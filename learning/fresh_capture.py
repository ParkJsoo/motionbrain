"""Verify opt-in post-request first-DMA evidence; never infer control freshness."""
import re


def validate_fresh_capture(receipt, source):
    if receipt.get("query", {}).get("fresh") != "1":
        return None
    headers = {str(k).lower(): str(v) for k,v in receipt.get("headers", {}).items()}
    requested = headers.get("x-motionbrain-capture-requested-us", "")
    discarded = headers.get("x-motionbrain-discarded-frames", "")
    if (headers.get("x-motionbrain-capture-mode") != "after_request"
            or not re.fullmatch(r"[1-9][0-9]{0,18}", requested)
            or not re.fullmatch(r"[0-3]", discarded)
            or source.get("sourceMetadataValid") is not True
            or type(source.get("capturedUs")) is not int or source["capturedUs"] <= int(requested)):
        raise ValueError("fresh_capture_contract_mismatch")
    return {"mode": "after_request", "requestedUs": int(requested),
            "discardedFrames": int(discarded)}
