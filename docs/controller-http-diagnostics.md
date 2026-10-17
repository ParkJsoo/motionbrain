# Bounded controller HTTP diagnostics

Use `tools/diagnose_controller_http.py` to preserve evidence of an intermittent
controller status failure without executing a motor command or accessing a camera.
It makes at most four sequential GET requests: `/status`, `/sequence`, `/actions`,
and `/action_outputs`. It does not discover devices, follow redirects, retry,
load curl configuration, authenticate, or start services.

Run it from the host whose network path is being investigated, with the current
controller address already verified. Each invocation requires a new evidence
directory; an existing session is never resumed or overwritten.

```sh
python3 tools/diagnose_controller_http.py \
  --target http://motionbrain.local \
  --output .local/http-diagnostic/session-01
```

The fixed collection limits are 0.8 seconds to connect, 2 seconds per HTTP
request, and a 12-second session budget. Any response taking more than 0.8 seconds
fails the diagnostic; the longer collection timeout is not a relaxed acceptance
criterion. Each response is limited to 64 KiB by curl and checked before parsing.
Curl must support `--max-filesize`; older curl versions may not enforce the limit
while receiving a response without Content-Length. Use a current curl for a
strict transfer-size bound. Host scheduling and filesystem delays may extend
wall-clock cleanup beyond the subprocess deadline.

`plan.json` records the target and limits before the first request. Each attempt
has the original response body (when curl produced one), stdout timing fields,
stderr, attempt timestamp, and a result. HTTP `000`, partial JSON, transport
failure, HTTP failure, invalid metrics, invalid payload, and interruption remain
distinguishable. A failure terminates collection immediately. Connect/first-byte/
total timings narrow the observed phase; they do not establish a root cause.

Success means the four responses arrived within the diagnostic limit and the
reported controller was idle, all five motors disabled at speed zero, the queue
empty, actions inactive, and the action/output boot IDs consistent. These are
sequential observations, not an atomic snapshot, electrical measurements, a
proof of reliable stopping during motion, or a long-term reliability test.
Historical output entries are retained without interpreting them as current PWM.

A closed diagnostic must not trigger an automatic retry or a physical experiment.
Review its evidence first and prepare a separate finite session if required.
These limits do not alter the video 100 ms, anchor 15 s, or storage 250 ms gates.
Diagnostic records are not training-eligible data. Keep raw records under
Git-ignored `.local/`; do not publish device details or private captures.

Offline regression tests:

```sh
python3 -m unittest discover -s tests -p 'test_controller_http_diagnostic.py'
```
