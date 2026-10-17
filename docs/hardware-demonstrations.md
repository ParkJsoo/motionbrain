# Raw hardware demonstrations

`tools/archive_hardware_demo.py SOURCE NEW_DESTINATION` snapshots private manual
robot evidence without opening a connection or issuing commands. Source phases
contain `step-01`, `step-02`, etc. Only `plan.json`, `execution/result.json`,
`after.png` or `after.jpg`, optional `camera.jpg` and `camera-receipt.json` are copied. Credentials,
scripts and unrelated files are excluded. Source paths must not contain symlinks;
keep source files unchanged during archival.

```sh
python3 tools/archive_hardware_demo.py .local/example-phase datasets/raw-demo-001 \
  --observation placed --note 'Object remained standing while open jaws lifted.'
```

The destination must be new. Files are immutable, private, fsynced and verified by
SHA-256 readback before terminal `demo.json` is written. Failed or missing attempts
are retained in the inventory. `archiveComplete` means copying finished;
`sourceRecordsComplete` means plan, result and at least one after image exist for every contiguous
step, not that execution or the physical task succeeded. Caught write failures remove the terminal manifest. Abrupt process/power loss
can interrupt cleanup: audit surviving bytes before use, and never resume an
archive in place. A later audit does not prove earlier fsync durability. `audit_demo()` detects
missing/changed bytes and changes to the raw-only contract; hashes do not protect
against an editor changing both files and manifest.

`--observation` is an operator visual annotation linked to the archived after
images. It is never inferred from ACK, STOP, or motor OFF. Source result JSON is
preserved byte-for-byte and may contain claims; those claims never grant this
archive eligibility. Images are observation snapshots, not synchronized action
edge frames. Host screenshot acquisition is not optical exposure time.

The existing requirements remain source age **100ms**, anchor age **15s**, and
source capture persistence **250ms**. They are policy values, not measured passes.
All three checks are `unknown`: offline archive write speed cannot prove source
capture persistence. `trainingEligible` and `controlEligible` are always false;
the distinct `motionbrain.hardware_demo.v1` format is not a capture-session input
for the training dataset builder. Use these records for visual annotation,
repeatability analysis and debugging. Qualifying transitions still requires the
existing optical timing, semantic anchor, bounded-action and persistence evidence.
Keep images, receipts and archives in ignored `.local/` or `datasets/`, never Git.

Image suffixes must match their encoded PNG/JPEG signatures. If a screenshot
provider returns JPEG under a PNG filename, prepare a byte-identical `.jpg` copy
in a separate input directory; do not transcode or discard the original evidence.
