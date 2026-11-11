#!/usr/bin/env python3
"""FileReviewer 전용 로컬 판독 화면. 촬영·장치 연결·모터 명령은 수행하지 않는다."""
from __future__ import annotations
import argparse
from http.server import HTTPServer
from pathlib import Path
import secrets
import sys
import urllib.parse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.file_review_console import FileReviewConsole
from tools.manual_collection_server import CollectionHandler, strict_json


class ReviewHandler(CollectionHandler):
    def do_GET(self):
        try:
            self.local_request()
        except ValueError as exc:
            return self.reply(403,dict(error=str(exc)))
        target = urllib.parse.urlsplit(self.path); query = urllib.parse.parse_qs(target.query)
        try:
            if target.path == '/':
                return self.reply(200,Path(__file__).with_name('file_review.html').read_bytes(),'text/html; charset=utf-8')
            if target.path == '/api/config':
                return self.reply(200,dict(token=self.server.ui_token))
            if target.path == '/api/state':
                return self.reply(200,self.server.manager.status())
            if target.path == '/api/preview':
                camera = query.get('camera',[''])[0]
                data = self.server.manager.preview(camera,query.get('request',[''])[0])
                return self.reply(200,data,'image/png' if camera=='external' else 'image/jpeg')
            return self.reply(404,dict(error='not_found'))
        except (ValueError,KeyError,OSError,RuntimeError):
            return self.reply(409,dict(error='원본이나 요청을 확인할 수 없습니다. 제출하지 않습니다.'))

    def do_POST(self):
        try:
            self.local_request()
            if not secrets.compare_digest(self.headers.get('X-Collection-Token',''),self.server.ui_token):
                raise ValueError('review_token_required')
        except ValueError as exc:
            return self.reply(403,dict(error=str(exc)))
        try:
            if self.path != '/api/decision': return self.reply(404,dict(error='not_found'))
            if self.headers.get('Transfer-Encoding') or self.headers.get_content_type() != 'application/json':
                raise ValueError('json_required')
            size = int(self.headers.get('Content-Length','-1'))
            if not 0<size<=8192: raise ValueError('bounded_body_required')
            result = self.server.manager.submit(strict_json(self.rfile.read(size)))
            return self.reply(200,result)
        except (ValueError,KeyError,OSError,RuntimeError):
            return self.reply(409,dict(error='제출하지 못했습니다. 제한시간·원본·명시한 판정을 확인하세요. 자동 재시도하지 않습니다.'))


class ReviewServer(HTTPServer):
    allow_reuse_address = True
    def __init__(self,address,manager):
        if address[0] != '127.0.0.1': raise ValueError('loopback_only')
        self.manager = manager
        self.ui_token = secrets.token_urlsafe(32)
        super().__init__(address,ReviewHandler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scope',type=Path,required=True)
    parser.add_argument('--request-root',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--read-only-review',action='store_true',help='모터 없는 판독 시험: 판정 불가 응답만 허용')
    args = parser.parse_args()
    manager = FileReviewConsole(ROOT,args.scope,args.request_root,read_only_review=args.read_only_review)
    with ReviewServer(('127.0.0.1',args.port),manager) as server:
        print(f'판독 화면: http://127.0.0.1:{server.server_port} (촬영·구동 없음)',flush=True)
        try: server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt: pass


if __name__ == '__main__': main()
