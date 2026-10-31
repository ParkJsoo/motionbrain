import ctypes
import hashlib
import json
import sys
import time
import unittest

from learning.lossless_frame import ENCODING, SUFFIX, library, decode
from tests.test_visual_camera_stream import CameraStreamTest


def encode(raw):
    src=ctypes.create_string_buffer(raw);dst=ctypes.create_string_buffer(len(raw)+65536)
    n=library().compression_encode_buffer(dst,len(dst),src,len(raw),None,0x702)
    if not n:raise ValueError('test_encode_failed')
    return dst.raw[:n]


@unittest.skipUnless(sys.platform=='darwin','Apple libcompression required')
class LosslessTest(CameraStreamTest):
    def compressed(self,**changes):
        receipt=self.frame();m=json.loads((self.root/'frame-0000.json').read_text())
        raw=(self.root/'frame-0000.bgra').read_bytes();stored=encode(raw)
        m.update(schemaVersion='motionbrain.mac_camera_frame.v2',storageEncoding=ENCODING,
                 rawByteLength=len(raw),storedByteLength=len(stored),pixelSha256=m['imageSha256'],
                 imageFile='frame-0000'+SUFFIX,imageSha256=hashlib.sha256(stored).hexdigest())
        m.update(changes);data=json.dumps(m).encode();(self.root/'frame-0000.json').write_bytes(data)
        (self.root/('frame-0000'+SUFFIX)).write_bytes(stored)
        receipt.update(metadataSha256=hashlib.sha256(data).hexdigest(),imageFile=m['imageFile'],imageSha256=m['imageSha256'])
        self.c.plan['storageEncoding']=ENCODING
        return receipt,raw,stored
    def test_exact_roundtrip_and_compressed_byte_budget(self):
        r,raw,stored=self.compressed();self.c.plan['maxBytes']=len(stored)
        self.c._packet(r,time.monotonic(),time.monotonic())
        self.assertEqual(self.c.pixels(self.c.latest),raw);self.assertEqual(self.c.total_bytes,len(stored))
        self.assertEqual(self.tracker.update.call_args.args[0],raw)
        self.assertEqual(self.c.latest['pixelSha256'],hashlib.sha256(raw).hexdigest())
    def test_raw_hash_or_size_corruption_stops_before_tracking(self):
        for changes in ({'pixelSha256':'0'*64},{'rawByteLength':999999999},{'storedByteLength':1}):
            r,_,_=self.compressed(**changes)
            with self.assertRaises(ValueError):self.c._packet(r,time.monotonic(),time.monotonic())
            self.tracker.update.assert_not_called()
    def test_compressed_frame_not_accepted_as_legacy_raw(self):
        r,_,_=self.compressed();self.c.plan.pop('storageEncoding')
        with self.assertRaises(ValueError):self.c._packet(r,time.monotonic(),time.monotonic())
    def test_bounded_decoder_rejects_truncated_wrong_size_and_wrong_hash(self):
        raw=bytes(range(256))*300;stored=encode(raw);h=hashlib.sha256(raw).hexdigest()
        self.assertEqual(decode(stored,len(raw),h),raw)
        for data,size,digest in [(stored[:-5],len(raw),h),(stored,len(raw)-4,h),(stored,len(raw),'f'*64),(stored,10**10,h)]:
            with self.assertRaises(ValueError):decode(data,size,digest)
