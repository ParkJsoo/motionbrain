import ctypes as C
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from learning.macos_credentials import load_keychain_secret

class KeychainTests(unittest.TestCase):
 def api(self,raw=b'synthetic-unit-secret',status=0):
  self.buffer=C.create_string_buffer(raw)
  def find(*args):
   if status:return status
   C.cast(args[5],C.POINTER(C.c_uint32))[0]=len(raw)
   C.cast(args[6],C.POINTER(C.c_void_p))[0]=C.addressof(self.buffer)
   C.cast(args[7],C.POINTER(C.c_void_p))[0]=123
   return 0
  sec=SimpleNamespace(SecKeychainFindGenericPassword=Mock(side_effect=find),SecKeychainItemFreeContent=Mock())
  cf=SimpleNamespace(CFRelease=Mock())
  return sec,cf
 def test_reads_only_explicit_item_and_releases_native_memory(self):
  sec,cf=self.api()
  with patch('learning.macos_credentials.sys.platform','darwin'),patch('learning.macos_credentials.C.CDLL',side_effect=[sec,cf]):
   self.assertEqual(load_keychain_secret('unit-service','unit-account'),'synthetic-unit-secret')
  sec.SecKeychainItemFreeContent.assert_called_once();cf.CFRelease.assert_called_once()
  self.assertEqual(sec.SecKeychainFindGenericPassword.call_args.args[2],b'unit-service')
 def test_decode_failure_still_releases_memory_without_printing_secret(self):
  sec,cf=self.api(b'\xff')
  with patch('learning.macos_credentials.sys.platform','darwin'),patch('learning.macos_credentials.C.CDLL',side_effect=[sec,cf]):
   with self.assertRaises(UnicodeDecodeError):load_keychain_secret('service','account')
  sec.SecKeychainItemFreeContent.assert_called_once();cf.CFRelease.assert_called_once()
 def test_missing_or_unsupported_items_fail_without_exposing_data(self):
  with patch('learning.macos_credentials.sys.platform','linux'):
   with self.assertRaisesRegex(RuntimeError,'macos_keychain_required'):load_keychain_secret('service','account')
  sec,cf=self.api(status=-25300)
  with patch('learning.macos_credentials.sys.platform','darwin'),patch('learning.macos_credentials.C.CDLL',side_effect=[sec,cf]):
   with self.assertRaisesRegex(RuntimeError,'keychain_status_-25300'):load_keychain_secret('service','account')

if __name__=='__main__':unittest.main()
