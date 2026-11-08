#!/usr/bin/env python3
"""준비한 비공개 자료만 loopback에서 제공하며 장치·클라우드에 연결하지 않는다."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.parse import unquote, urlsplit

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from learning.robot_visualization import publication_matches


def make_handler(root, rerun_port=9090):
    root=Path(root).resolve()
    class Handler(SimpleHTTPRequestHandler):
        def allowed(self):
            decoded=unquote(urlsplit(self.path).path)
            target=(root/decoded.lstrip('/')).resolve()
            if decoded=='/':target=root/'index.html'
            if target==root/'wandb-publication.json':
                try:
                    if not publication_matches(json.loads((root/'experiment.json').read_text()),json.loads(target.read_text())):return False
                except (ValueError,KeyError,TypeError,OSError):return False
            exact={root/name for name in ('index.html','experiment.json','public-metrics.json','evaluation.rrd','wandb-publication.json')}
            media=(root/'media').resolve()
            return target.is_file() and (target in exact or
                (target.is_relative_to(media) and target.suffix.lower() in ('.png','.jpg','.jpeg')))
        def do_GET(self):
            if not self.allowed():self.send_error(404);return
            super().do_GET()
        def do_HEAD(self):
            if not self.allowed():self.send_error(404);return
            super().do_HEAD()
        def list_directory(self,path):
            self.send_error(404);return None
        def end_headers(self):
            self.send_header('Access-Control-Allow-Origin',f'http://127.0.0.1:{rerun_port}')
            super().end_headers()
        def log_message(self,*args):pass
    return partial(Handler,directory=str(root))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--port',type=int,default=8790)
    a=p.parse_args()
    root=a.directory.resolve()
    if not (root/'experiment.json').is_file():p.error('prepare.py로 준비한 폴더가 필요합니다')
    server=ThreadingHTTPServer(('127.0.0.1',a.port),make_handler(root))
    # The SDK asset server has no bind option; the CLI supports loopback binding.
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',9090))==0:
            server.server_close();p.error('9090 포트가 사용 중입니다. 기존 서버를 보존했습니다')
    child=subprocess.Popen([str(Path(sys.executable).with_name('rerun')),'--serve-web',
        '--bind','127.0.0.1','--web-viewer-port','9090','--port','9876'],
        env=dict(os.environ,RERUN_ANALYTICS_ENABLED='false'))
    try:
        for _ in range(100):
            if child.poll() is not None:raise RuntimeError('Rerun 서버 시작 실패')
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1',9090))==0:break
            time.sleep(.1)
        else:raise RuntimeError('Rerun 서버 시작 시간 초과')
        print(f'http://127.0.0.1:{a.port}',flush=True)
        server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.server_close()
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()


if __name__=='__main__':main()
