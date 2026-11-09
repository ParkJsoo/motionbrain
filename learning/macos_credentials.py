"""macOS 키체인의 명시한 항목을 메모리로만 읽는다. 출력·파일 저장 없음."""
import ctypes as C
import sys


def load_keychain_secret(service,account):
    if sys.platform!='darwin':raise RuntimeError('macos_keychain_required')
    for value in (service,account):
        if not isinstance(value,str) or not 1<=len(value.encode())<=512:raise ValueError('explicit_keychain_item_required')
    service,account=service.encode(),account.encode()
    security=C.CDLL('/System/Library/Frameworks/Security.framework/Security')
    foundation=C.CDLL('/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation')
    find=security.SecKeychainFindGenericPassword
    find.argtypes=[C.c_void_p,C.c_uint32,C.c_char_p,C.c_uint32,C.c_char_p,C.POINTER(C.c_uint32),C.POINTER(C.c_void_p),C.POINTER(C.c_void_p)]
    find.restype=C.c_int32
    free=security.SecKeychainItemFreeContent;free.argtypes=[C.c_void_p,C.c_void_p];free.restype=C.c_int32
    release=foundation.CFRelease;release.argtypes=[C.c_void_p]
    length=C.c_uint32();data=C.c_void_p();item=C.c_void_p()
    status=find(None,len(service),service,len(account),account,C.byref(length),C.byref(data),C.byref(item))
    if status!=0:raise RuntimeError('keychain_status_'+str(status))
    try:
        if not data.value or not 1<=length.value<=4096:raise ValueError('keychain_secret_length_invalid')
        return C.string_at(data,length.value).decode()
    finally:
        if data.value:free(None,data)
        if item.value:release(item)
