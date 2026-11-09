#!/usr/bin/env python3
"""고정한 M1 계획의 명시적 단발 서버. 비공개 입력 또는 OS 키체인 인증."""
import argparse,getpass,json,signal,sys
from pathlib import Path
from http.server import HTTPServer,BaseHTTPRequestHandler
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from learning.m1_shaft_campaign import M1ShaftCampaign
from learning.macos_credentials import load_keychain_secret


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8769)
    parser.add_argument('--keychain-service');parser.add_argument('--keychain-account')
    args=parser.parse_args();config=args.config.resolve(strict=True)
    if not config.is_relative_to((ROOT/'.local').resolve()):parser.error('비공개 계획 경로가 필요합니다')
    if not 1024<=args.port<=65535:parser.error('유효한 localhost 포트가 필요합니다')
    if bool(args.keychain_service)!=bool(args.keychain_account):parser.error('키체인 service와 account를 함께 지정합니다')
    if args.keychain_service:
        credential=lambda:load_keychain_secret(args.keychain_service,args.keychain_account)
    else:
        if not sys.stdin.isatty():parser.error('비공개 TTY 또는 명시한 키체인 항목이 필요합니다')
        credential=lambda:getpass.getpass('장치 인증값(메모리에서만 사용): ')
    value=json.loads(config.read_bytes());campaign=None;server=None
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        campaign=M1ShaftCampaign(ROOT,config.parent,value,credential)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*_):pass
            def reply(self,code,value):
                raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode();self.send_response(code)
                self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
            def do_GET(self):self.reply(200,campaign.state())
            def do_POST(self):
                try:
                    if (self.path!='/step' or self.headers.get('Origin') is not None
                        or self.headers.get('Host')!=f'127.0.0.1:{args.port}'
                        or self.headers.get('Content-Type')!='application/json'):raise ValueError('explicit_local_json_step_required')
                    self.connection.settimeout(3);size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=512:raise ValueError('bounded_step_request_required')
                    request=json.loads(self.rfile.read(size))
                    if set(request)!={'index','planSha256'} or request['planSha256']!=campaign.plan_sha:raise ValueError('fixed_plan_required')
                    self.reply(200,campaign.step(request['index']))
                except Exception as exc:self.reply(409,dict(reason=str(exc),errorType=type(exc).__name__,state=campaign.state()))
        server=HTTPServer(('127.0.0.1',args.port),Handler)
        print(json.dumps(dict(status='ready',port=args.port,planSha256=campaign.plan_sha,automaticExecution=False)),flush=True)
        server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        if server:server.server_close()
        if campaign:campaign.close()

if __name__=='__main__':main()
