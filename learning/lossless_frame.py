"""Bounded decoding of macOS LZBITMAP BGRA frames, without camera access."""
import ctypes
import hashlib
from functools import lru_cache

ENCODING = 'apple-lzbitmap-bgra8-v1'
SUFFIX = '.bgra.lzbitmap'


@lru_cache(maxsize=1)
def library():
    lib = ctypes.CDLL('/usr/lib/libcompression.dylib')
    for name in ('compression_encode_buffer', 'compression_decode_buffer'):
        f = getattr(lib, name)
        f.restype = ctypes.c_size_t
        f.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p,
                      ctypes.c_size_t, ctypes.c_void_p, ctypes.c_uint32]
    return lib


def decode(stored, raw_bytes, pixel_sha256):
    if type(raw_bytes) is not int or not 4 <= raw_bytes <= 1920*1080*4 or not 0 < len(stored) <= raw_bytes+65536:
        raise ValueError('compressed_frame_size_invalid')
    src = ctypes.create_string_buffer(stored)
    dst = ctypes.create_string_buffer(raw_bytes+1)
    count = library().compression_decode_buffer(dst, len(dst), src, len(stored), None, 0x702)
    raw = dst.raw[:count]
    if count != raw_bytes or hashlib.sha256(raw).hexdigest() != pixel_sha256:
        raise ValueError('compressed_frame_pixels_invalid')
    return raw
