# 물리 Safety Evidence 보강 계획

[English](physical-safety-validation-plan.en.md)

## 상태

이 문서는 planned evidence다. 아래 항목은 아직 완료 증거가 아니며, public
claim-to-evidence matrix의 완료 Evidence로 승격하지 않는다. 각 항목은 지정된
artifact가 캡처되고 pass/fail 판정이 리뷰된 뒤에만 "captured" 또는 "passed"로
상태를 바꿀 수 있다.

이 계획이 보강하려는 gap은 다음과 같다.

- hard E-stop 또는 TB6612FNG `VM` cutoff가 실제 motor power를 끊는지
- deadman release부터 ESP32 PWM off까지의 지연 시간
- ESP32 PWM, STM32-ESP32 UART, M4 AS5600 I2C 파형
- motor supply voltage sag와 controller/sensor 영향

이 계획은 certified functional safety, unattended operation, production power
integrity를 주장하지 않는다.

## 공통 산출물 규칙

모든 캡처는 새 evidence note로 승격하기 전까지 아래 naming을 따른다.

```text
docs/evidence/artifacts/physical-safety/YYYY-MM-DD/
  YYYY-MM-DD-physical-safety-<test-id>-runNN-summary.md
  YYYY-MM-DD-physical-safety-<test-id>-runNN.logic.sr
  YYYY-MM-DD-physical-safety-<test-id>-runNN.scope.csv
  YYYY-MM-DD-physical-safety-<test-id>-runNN.scope.png
  YYYY-MM-DD-physical-safety-<test-id>-runNN.serial.log
  YYYY-MM-DD-physical-safety-<test-id>-runNN.video.mp4
  YYYY-MM-DD-physical-safety-<test-id>-runNN-photo.jpg
```

`summary.md`에는 다음을 기록한다.

- hardware revision, battery or bench-supply setting, load condition, firmware
  build identity
- probe points, probe ground, instrument model/sample rate
- command sequence and operator action
- measured values, pass/fail result, failed acceptance item if any
- final state: motor speeds 0, fault/safety state, sensor status

원본 `.sr`, `.csv`, `.log`, `.mp4`는 요약 PNG보다 우선한다. PNG와 표만 있고
원본 캡처가 없으면 public Evidence로 쓰지 않는다.

## Evidence Status Map

| ID | Planned claim after pass | Current status | Required evidence | Still not claimed |
| --- | --- | --- | --- | --- |
| `P1-HW-ESTOP` | Hardware E-stop or `VM` cutoff removes motor drive power during bounded motion | planned | wiring photo/schematic, switch edge, motor-side `VM`, commanded PWM, video, final state log | Certified E-stop, safety-rated channel, unattended operation |
| `P1-DEADMAN-LATENCY` | Deadman release drives active motor PWM to off within the measured acceptance window | planned | synchronized deadman edge, active PWM pin, optional UART decode, serial/status log, repeated runs | Human-safe stop distance, certified stop category |
| `P1-PWM-UART-I2C` | Bench waveforms match expected PWM frequency/duty, UART framing, and I2C health under bounded commands | planned | logic/scope captures, protocol decodes, screenshots, command log | EMC margin, all-environment signal integrity |
| `P1-MOTOR-SAG` | Bounded M4 motion keeps motor supply within measured bench acceptance and does not reset controller/sensors | planned | simultaneous supply traces, command log, sensor/status log, final state | Root-cause closure for all power faults, battery lifetime guarantee |

## P1-HW-ESTOP: Hard E-stop 또는 VM Cutoff

### Requirement

물리 stop evidence는 firmware `stop`, HTTP command, ROS2 guard만으로 충분하지
않다. 최소 하나의 hard E-stop 또는 `VM` cutoff 경로가 TB6612FNG motor supply
path를 끊는다는 증거가 필요하다. 설치된 hardware cutoff가 없다면 이 항목은
`blocked: hardware not installed`로 남긴다.

### Procedure

1. 로봇팔을 지그 또는 손이 닿지 않는 bench 위치에 고정한다.
2. TB6612FNG motor-side `VM`, E-stop/contact edge, active motor PWM을 동시에
   캡처한다. 가능하면 motor terminal voltage 또는 supply current도 함께 캡처한다.
3. 저속/짧은 bounded M4 motion을 시작한다.
4. motion 중 hard E-stop 또는 `VM` cutoff를 작동한다.
5. release/reset 전에는 새 motion command가 실행되지 않는지 확인한다.
6. reset/re-arm 절차를 별도 로그로 남긴다.

### Required Artifacts

- `p1-hw-estop-runNN-photo.jpg`: cutoff wiring and probe placement
- `p1-hw-estop-runNN.logic.sr`: switch/contact edge and active PWM
- `p1-hw-estop-runNN.scope.csv`: motor-side `VM` voltage trace
- `p1-hw-estop-runNN.scope.png`: annotated timing screenshot
- `p1-hw-estop-runNN.video.mp4`: operator action and motor stop
- `p1-hw-estop-runNN.serial.log`: final controller status and re-arm behavior

### Pass/Fail Criteria

Pass:

- cutoff is hardware-visible as a switch/contact edge or relay/MOSFET gate event
- motor-side `VM` is disconnected or falls below 10% of pre-cut voltage within
  250 ms; if bulk capacitance keeps voltage above that, the summary must show
  no H-bridge drive current/path and mark residual-energy handling as follow-up
- active PWM is off or irrelevant because `VM` is physically removed
- no motor continues driven motion after cutoff
- restart requires explicit reset/re-arm; motion does not resume automatically

Fail:

- only firmware/software stop is shown
- `VM` remains connected and motor can still be driven after cutoff
- motion resumes without explicit reset/re-arm
- artifacts do not show both operator action and electrical cutoff timing

## P1-DEADMAN-LATENCY: Release-to-PWM-Off

### Requirement

Deadman release evidence must measure the time from a physical release signal to
PWM off on the active motor path. Video alone is not enough unless synchronized
with an electrical timestamp.

### Procedure

1. Capture the deadman switch edge or STM32 release frame timing, ESP32 active
   PWM pin, and serial/status log in one run.
2. Use a bounded low-speed M4 command or teleoperation motion.
3. Release deadman while PWM is active.
4. Repeat at least 10 times across the same setup.
5. Record release edge time, last non-zero PWM edge/duty, final motor speed 0,
   and final controller state.

### Required Artifacts

- `p1-deadman-latency-runNN.logic.sr`: release edge and PWM off timing
- `p1-deadman-latency-runNN.scope.png`: annotated latency cursor screenshot
- `p1-deadman-latency-runNN.serial.log`: state/event output
- `p1-deadman-latency-runNN.video.mp4`: physical release context
- `p1-deadman-latency-summary.md`: table of all runs, mean, p95, max

### Acceptance

Pass:

- at least 10/10 release trials captured with synchronized electrical timing
- active PWM duty reaches 0 within 100 ms max from release edge
- no non-commanded axis produces PWM during or after release
- final status reports all motor speeds 0 and no automatic restart

Fail:

- max release-to-PWM-off latency exceeds 100 ms
- PWM reappears without a new operator command
- only frame timeout is measured while the physical release event is missing
- serial/status output contradicts the captured electrical state

If the system intentionally relies on a configured frame timeout instead of an
explicit release frame, this item remains failed for deadman-release evidence and
must be split into a separate timeout evidence note.

## P1-PWM-UART-I2C: 파형 캡처

### Requirement

Runtime logs and DMM checks do not prove waveform timing. This evidence requires
logic analyzer or oscilloscope captures with raw files and decoded protocol
output.

### PWM Procedure

1. Pick one active axis first, preferably M4 because it has AS5600 feedback.
2. Probe its PWM pin and both direction pins using the current pin map.
3. Run bounded commands at representative duty levels, such as 25%, 50%, 75%,
   and 100%, if mechanically safe.
4. Confirm non-commanded motor PWM pins remain idle when practical.

PWM pass criteria:

- measured PWM frequency is 1 kHz +/- 5% or matches the documented configured
  firmware value
- duty is within +/- 5 percentage points of the commanded duty, or the summary
  explains 8-bit rounding
- direction pins are not both active in a shoot-through state
- non-commanded axes stay at 0 PWM during the bounded command

### UART Procedure

1. Probe STM32-to-ESP32 UART at the ESP32 receive side.
2. Decode as 115200 8N1 unless the active firmware config says otherwise.
3. Capture idle, active teleoperation, deadman release, and timeout/stop cases.

UART pass criteria:

- decoder reports the configured baud/framing with no framing errors in the
  captured window
- frame cadence is recorded and has no unexplained gap larger than 2x nominal
  cadence during active teleoperation
- release or stop frames are visible when the procedure depends on them

### I2C Procedure

1. Probe M4 AS5600 I2C at ESP32 SDA `GPIO0` and SCL `GPIO15`; do not probe a Pi
   I2C bus as a substitute.
2. Capture `shoulder status` and at least one bounded shoulder target.
3. If STM32 MPU-6050 I2C is captured, keep it as a separate run and do not attach
   probes that can disturb boot straps or pull-ups.

I2C pass criteria:

- SDA/SCL idle high before transaction
- expected address ACK is visible (`0x36` for AS5600, `0x68` for MPU-6050 if
  captured)
- no stuck-low bus, repeated NACK storm, or unexpected bus reset during motion
- rise time meets the selected I2C mode limit; if the mode is unknown, record
  bus speed and compare against standard-mode 100 kHz / 1000 ns rise-time
  expectations as the conservative baseline

### Required Artifacts

- `p1-waveform-pwm-runNN.logic.sr`, `.scope.png`, `.scope.csv`
- `p1-waveform-uart-runNN.logic.sr`, decode export, `.serial.log`
- `p1-waveform-i2c-as5600-runNN.logic.sr`, decode export, `.scope.png`
- command transcript and final status log for each run

## P1-MOTOR-SAG: Motor Voltage Sag

### Requirement

Phone video plus a basic multimeter can show approximate voltage drop, but P1
evidence needs synchronized traces that separate battery terminal voltage,
regulator or XL4015 output, and TB6612FNG motor-side `VM` as much as practical.

### Procedure

1. Use a two-channel oscilloscope or synchronized data logger. If only meters are
   available, mark status as `captured: approximate`, not `passed`.
2. Capture battery terminal voltage and TB6612FNG motor-side `VM`; add XL4015
   output as a third channel if available.
3. Run bounded M4 motion with no added load and with the known 23.10 g load.
4. For each run, record pre-motion voltage, minimum voltage, recovery voltage at
   1 s after motion, controller status, AS5600 status, and STM32 IMU status.
5. Repeat at least 3 runs per condition.

### Required Artifacts

- `p1-motor-sag-noload-runNN.scope.csv`
- `p1-motor-sag-23g-runNN.scope.csv`
- `p1-motor-sag-runNN.scope.png`
- `p1-motor-sag-runNN.serial.log`
- `p1-motor-sag-runNN.video.mp4`
- `p1-motor-sag-summary.md`: table with pre/min/recovery voltage and percent sag

### Acceptance

Pass for bounded bench evidence:

- 3/3 no-load and 3/3 23.10 g runs captured with synchronized voltage traces
- motor-side `VM` stays above the project-defined motor-supply minimum; until a
  stricter hardware limit is documented, any drop below 5.0 V on the nominal 6 V
  bench supply is a fail for this evidence item
- no ESP32 brownout/reset, STM32 IMU fault, AS5600 readiness fault, or unexpected
  SafetyGate block occurs during the captured pulse
- voltage recovers to within 95% of pre-motion value within 1 s after the motion
  ends

Fail:

- only approximate handheld readings are available
- voltage and command timing are not synchronized
- controller or sensor resets during the bounded pulse
- sag exceeds the acceptance threshold without a follow-up mitigation note

## Evidence Promotion Rule

After capture, create a dated evidence note only if the raw artifacts are present
and the pass/fail table is complete. Until then, public documents may say only:

```text
Physical safety instrumentation plan exists for hard cutoff, deadman latency,
PWM/UART/I2C waveform, and motor-supply sag evidence; these are planned, not yet
completed evidence.
```

Do not convert this plan into claims such as certified functional safety,
production E-stop, verified stop distance, or production power integrity.
