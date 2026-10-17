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

## Deployment and remaining checks

Firmware has not been uploaded; device behavior with this patch is unverified. User approval is required before uploading/restarting the ESP32 controller. Keep the existing Chrome dashboard/control/camera and QuickTime preview open. After an approved upload, verify the new boot and OFF state, repeat the same bounded empty-client fixture, confirm the release counter, and check ordinary GET/form/query traffic and authenticated non-motion commands under the unchanged criterion. If unexplained timeouts remain, continue packet-level diagnosis instead of declaring the communication issue fixed or resuming grasp commands.

The robot remains near the lipbalm in the last visually observed pose. Existing failed physical trials stay closed; any later motion must use a fresh admitted plan and verify output/state and actual movement. No gripper-close or successful grasp is claimed.
