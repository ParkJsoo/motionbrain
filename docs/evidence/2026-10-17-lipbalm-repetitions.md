# Lipbalm place and repeated demonstrations — 2026-10-17

Following the first visually confirmed grasp, the user authorized placing the
lipbalm and repeating the operation to begin data collection. The object was
placed upright, then grasped, lifted and placed in two additional observed cycles.
A separate read-only reviewer checked the image sequences and controller results.

| Phase | Motion requests | Visual outcome |
| --- | ---: | --- |
| Initial place | 5 | Lowered, opened, raised empty jaws; object remained standing |
| First repeat | 8 + next phase's first release check | First lift missed; one extra close pulse allowed visible lift, then placement and release |
| Second phase | 8, including prior release check | Planned two close pulses, visible lift, lower, open, raise empty jaws; no extra close correction |

These phase counts total **21 new motion requests**, not 22: the first step of the
second phase is the release verification shared with the first cycle. This is
**two additional grasp/lift/place demonstrations**, one requiring correction.
It is not a measured autonomous-policy success rate or uncorrected repeatability
claim. No encoder-calibrated lift height or gripping force is inferred.

Every new request finished with `failure=null`, verified IDLE, five motors OFF
and an empty queue. The final command was STOP232. A separate four-endpoint
read-only check confirmed unchanged STOP, controller boot and output history;
maximum request time was 65.7ms and shoulder angle was 219.90 degrees. The same
PWM limits, 750ms manual lease, 0.8s request criterion, output audit and per-step
visual inspection were retained. No firmware upload or service restart occurred.
This finite sample does not eliminate the previously observed HTTP latency risk.

## Raw collection

Three private immutable archives contain 21 plans/results, 21 iPhone observation
snapshots, 16 ESP32-CAM JPEGs and their 16 HTTP receipts. All archive audits passed.
The initial-place phase predates paired CAM snapshots, and this missing modality
is explicit. The first repeat's missed lift is retained rather than discarded.
The iPhone provider returned JPEG bytes under locally chosen `.png` names; original
files were kept, and byte-identical `.jpg` copies were prepared for archival with
SHA-256 correspondence. The initial mislabeled import failed and has no terminal
manifest; it is excluded from the three complete archives.

**Training-eligible transitions remain zero.** These are unsynchronized observation
snapshots plus manual-lease control logs. Source optical age, semantic anchor age
and source capture persistence are unverified. Video100ms, anchor15s and storage250ms
are unchanged; fast offline copying does not qualify original capture persistence.
The raw-only schema cannot be passed off as a validated capture-session dataset.
Raw images, tokens, device addresses and private receipts are not committed.

## Software verification

- `hardware_demo`: 12 tests, including failed terminal readback cleanup, byte
  preservation, missing/failed attempts, symlink rejection, tampering, JPEG
  signatures and existing v1 PNG inventory compatibility.
- `test_action_dataset`: 15 passed; `test_transition_artifacts`: 13 passed.
- Independent implementation review identified and resolved terminal-failure,
  timing-contract and backward-compatibility issues; no remaining P0/P1/P2 finding.
- An additional `test_capture_persistence` invocation could not import `cv2` in
  the system Python. It did not execute and is not counted as a pass. No existing
  capture/persistence implementation was changed by this archive-only work.
- The work branch was pushed; no CI run was reported for the implementation commits.
  Main remains unchanged. Local tests and visual checks are not CI or training admission.

The next collection milestone is synchronized, qualified transition capture with
verified optical timestamps, semantic lipbalm anchors and the existing bounded
execution contract; raw demonstration replay alone does not satisfy it.
