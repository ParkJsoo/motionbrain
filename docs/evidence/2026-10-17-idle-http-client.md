# Empty HTTP connections and controller timeouts — 2026-10-17

## Observation

A visual grasp attempt stopped before ARM or a joint command when the default-speed POST exceeded the existing 0.8 s request limit. Its STOP response succeeded; strict command provenance could not account for the missing POST response. Subsequent telemetry showed IDLE, all five motors OFF, and no new nonzero motor outputs. No grasp was completed.

The follow-up diagnostics sent no ARM or motor commands:

- 32 counterbalanced pairs of unauthenticated POST requests to a fixed nonexistent route completed within 0.8 s. Form-body maximum was 141.430 ms; query/empty-body maximum was 96.322 ms. This does not validate authenticated command handling.
- A later initial GET /status connected in 158.874 ms, sent immediately, then timed out while waiting for response headers. Total was 2161.848 ms with a 2 s diagnostic collection timeout. The original 0.8 s acceptance criterion was not relaxed. This occurred before the planned idle connection was created.
- In a separate bounded trial, one empty TCP connection was held for 1.2 s. A status GET started 150 ms later failed with connection reset after 1415.0 ms. The initial and final state were IDLE with unchanged command identity, boot identity and output history. Final telemetry succeeded.

The private idle fixture initially compared the entire lastCommand object, including changing ageMs, and therefore reported unchangedIdleVerified=false. Comparing seen/id/type/source/executedAtMs/success instead confirmed unchanged identity; raw results are preserved. A previous fixture's failed initial GET also left missing baseline variables; no idle connection was opened in that run. These fixture errors are not controller failures.

A further 32-pair comparison with packet metadata also passed (form maximum 89.176 ms, query maximum 101.113 ms; capture alive until cleanup, 941 packets captured, zero kernel drops). A separate empty-connection fixture reproduced the failure with packet metadata: the queued GET connected in about 6.7 ms, sent its 142-byte request, and received an ACK for all request bytes about 238 ms later. No HTTP response followed. After the empty connection closed at 1.2 s, the GET received a reset; its total elapsed time was 1198.4 ms. The capture reported 127 packets and zero kernel drops. Final state, command identity, boot and motor-output history were unchanged.

This trace supports a server-side waiting/connection-handling problem in the empty-connection scenario; it does not establish the sole cause of the original intermittent failure, or exclude every wireless contribution. The first packet collectors failed before sending requests because their detached process session could not use the terminal's sudo ticket. Preserving the existing terminal session enabled capture. No raw packet capture, credential headers or unrelated UI connections were recorded. Text metadata includes the fixed probe request line; filters used dedicated probe source ports.

## Change

Arduino WebServer 2.0.17 accepts one current client and can wait 5000 ms for its first HTTP byte. The project-local IdleConnectionWebServer adapter releases an accepted client after 100 ms only when its state is HC_WAIT_READ and no byte is available. The base implementation clears the disconnected client's context; the next loop can accept another client. The listener, route handlers, authentication, request parser and motor lease logic are unchanged. No shared PlatformIO dependency is patched.

GET /routine now exposes diagnostics.http.firstByteWaitMs and diagnostics.http.idleConnectionReleases. The counter records this particular release path without adding per-connection serial logging.

This is not a complete HTTP latency bound: partial headers/bodies and response writes still use the SDK parser and timeouts. A legitimate connection that delays its first byte past 100 ms can be closed. Control commands must not automatically retry after an uncertain response. Video 100 ms, anchor 15 s, storage 250 ms and the 0.8 s command criterion remain unchanged; these diagnostics are not training-eligible.

## Software validation

- ESP32 release build passed using Espressif32 6.12.0 / Arduino 2.0.17 (RAM 65048 bytes, flash 1104289 bytes).
- 58 related tests passed: the new native adapter test, motor-output and bounded-action/service tests, dashboard authentication, guarded-routine contracts and HTTP diagnostics. Tests requiring loopback servers were rerun with network permission after sandbox bind failures.
- Native adapter coverage includes 99/100 ms boundary, an already-readable request, disconnected/closing clients, millis wraparound, queued real requests and consecutive empty clients. It uses a minimal transport double; the ESP32 build verifies compatibility with the actual SDK.
- Independent read-only review found no P0/P1/P2 issue in the final implementation and surrounding SDK cleanup path.

## Approved deployment and hardware validation

The user explicitly approved one main ESP32 firmware upload and confirmed the arm battery was OFF. Commit `2a98159` was uploaded once to the reidentified CP2102N controller. Upload completed in 64.51 s, flash hashes verified, and the controller rebooted. The application image SHA-256 was `447b75f5d7be022f17b2b8ab412f062f7a2cb40fe26efd070e790ad097796cfd`. Pi services, CAM firmware and STM32 firmware were not restarted or uploaded.

With the new boot, all five motors OFF and the existing browser tabs open:

- The same 1.2 s empty-client fixture completed its queued status GET in 39.4 ms, compared with the pre-change 1198.4 ms/reset result. The idle-release counter increased from 1 to 2. This global counter does not uniquely attribute a release to the fixture socket or prove an exact physical 100 ms deadline.
- Initial and final command identity, boot and motor-output history were unchanged. All nine fixture requests were below 0.8 s; the routine pre-read was 44.8 ms. The fixture's routine pre-read timing was inspected afterward because its admission check did not yet include that request.
- Twelve authenticated default-speed100 requests all succeeded, with maximum POST latency 78.947 ms. Maximum across these requests and surrounding telemetry was 359.034 ms. Final IDLE, unchanged motor outputs and exact command progression were verified. No ARM or joint request was made in this validation.
- These results validate the reproduced empty-client path and a finite normal-command sample, not all possible communication failures.

## Residual failure during the subsequent visual grasp attempt

After the user confirmed the battery was ON again, a fresh admitted visual approach was executed. On the eleventh motion request after the firmware change, a second gripper-close pulse received its command response and a successful STOP response, but the following final `/status` GET timed out after about 830 ms. That physical phase was closed and was not automatically resumed.

The move response took about 43 ms and STOP took about 150 ms. A separate read-only check confirmed IDLE, all five motors OFF, queue0, unchanged boot and STOP command identity. Retrospective output audit found only the intended M1 close output, lasting 442 ms. The original final-check failure is preserved separately from this later confirmation. The lipbalm was still standing; no lift or successful grasp was verified.

Thus the empty-client correction is effective for its reproduced scenario, but intermittent HTTP latency remains unresolved. A further bounded GET-only packet trace was started with all motors OFF. No threshold was relaxed and no training-eligible data was produced. Firmware changes require separate approval before any further upload.

The work branch contains the reviewed correction; main has not been merged or pushed. Branch-push CI is not configured: workflows run on main/master pushes, pull requests or manual dispatch. Local test/build success is not CI success.

## Route comparison and continued visual control

A finite GET-only comparison kept the 0.8 s criterion, 2 s diagnostic collection timeout, zero retries and existing UI tabs. Both Pi runs ended at the predeclared 80 s session budget, rather than a network timeout. All completed requests passed and final IDLE, command identity, boot and output history were unchanged.

| Route | Status requests including initial/final | Median / p95 / max total ms | Median / p95 / max connect ms |
| --- | ---: | ---: | ---: |
| Pi, original Wi-Fi power save on | 107 | 155.9 / 364.2 / 390.1 | 76.7 / 330.0 / 333.2 |
| Pi, temporary power save off | 109 | 127.9 / 362.4 / 394.7 | 75.2 / 329.3 / 359.5 |
| Mac direct, subsequent finite sample | 42 | 101.5 / 146.8 / 157.3 | 71.5 / 81.8 / 87.4 |

Pi power save off did not materially improve the observed delay and was restored to on. The official Ubuntu iw package was installed for this diagnosis; no service restart was needed. Pi packet collectors reported zero kernel drops. These sequential samples do not isolate an AP, ESP32 or driver root cause, and do not prove that the residual timeout is eliminated.

The Mac direct route then passed twelve authenticated default-speed100 requests, with a maximum of 103.638 ms across requests and surrounding telemetry. Command progression was exact and outputs remained unchanged. A fresh admission and an independently reviewed private runner used the same 0.8 s limit, 750 ms manual lease, per-step STOP, output/boot/provenance checks and PWM caps. Seventeen mock tests passed, including rejection of malformed credential values without echoing them. Temporary credentials were excluded from evidence archives.

Eleven subsequent single-joint requests through the Mac route completed with final IDLE and output audits. Shoulder, elbow and wrist adjustments advanced the approach; three short lift checks did not lift the lipbalm. One close request targeted PWM40 but only reached PWM20 because the output ramp began late; its move response took 694.3 ms and nonzero output lasted 302 ms. This is not evidence that PWM40 lacks gripping force. A separate close reached PWM40 and held it for 520 ms, followed by another observed close and lift check. No PWM cap was raised. Some later request latencies approached the 0.8 s criterion even on the Mac route, so it remains an alternate path with finite validation, not a complete communication fix.

At the last recorded stop, all five motors were OFF, the queue was empty and shoulder angle was 218.23 degrees. The lipbalm was still standing. The rear jaw was occluded in the iPhone view and the close camera could not establish whether the object lay between both jaws; one external depth observation was requested before further approach. No successful grasp or training-eligible episode is claimed. Video100 ms, anchor15 s and storage250 ms remain unchanged.
