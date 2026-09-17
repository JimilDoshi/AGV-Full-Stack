// =============================================================================
// ESP32 SKID-STEER UGV CONTROLLER  v2.1.0
// Hardware : ESP32 DevKit V1
//            FlySky FS-iA10B receiver (iBUS on GPIO16)
//            4× ROBU 24 V 250 W brushed motor controllers
//            4× hub motors (left pair / right pair)
//
// Uncomment to run logic only — no DAC or relay outputs. Safe for desk test.
// =============================================================================
//#define SIMULATION_MODE

#define FW_VERSION "2.2.0"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>          // ESP32 hardware watchdog
#include "driver/twai.h"

// Watchdog timeout — if loop() stalls longer than this, ESP32 resets.
#define WDT_TIMEOUT_S  2
#define WDT_TIMEOUT_MS (WDT_TIMEOUT_S * 1000U)

// =============================================================================
// CONFIGURATION — all tuneable values in one place
// =============================================================================

// Loop
const uint16_t LOOP_MS             = 20;      // 50 Hz

// iBUS receiver
const uint8_t  IBUS_GPIO           = 16;
const uint8_t  IBUS_CH             = 10;
const uint16_t IBUS_MIN            = 1000;
const uint16_t IBUS_MAX            = 2000;
const uint16_t IBUS_MID            = 1500;
const uint32_t FAILSAFE_MS         = 500;     // signal-loss → failsafe after this

// CAN (ESP32 built-in TWAI controller + external CAN transceiver)
const gpio_num_t CAN_TX_PIN        = GPIO_NUM_5;
const gpio_num_t CAN_RX_PIN        = GPIO_NUM_4;
const uint32_t CAN_CMD_ID          = 0x101;
const uint32_t CAN_STATUS_ID       = 0x200;
const uint32_t CAN_FAULT_ID        = 0x300;
const uint32_t CAN_TIMEOUT_MS      = 200;
const uint32_t CAN_SEQUENCE_TIMEOUT_MS = 200; // sequence must advance at 50 Hz
const uint16_t CAN_REARM_ZERO_MS   = 400;     // stable zero command before CAN arm
const int16_t  CAN_REARM_ZERO_LIMIT = 25;     // normalized command deadband
const uint8_t  CAN_MAGIC           = 0xA5;
const uint8_t  CAN_MODE_JETSON     = 1;
const uint8_t  CAN_MODE_AUTONOMOUS = 2;

// Channel indices (0-based)
const uint8_t  CH_STEER            = 0;       // CH1  left / right
const uint8_t  CH_DIR              = 1;       // CH2  forward / reverse stick
const uint8_t  CH_THR              = 2;       // CH3  speed
const uint8_t  CH_ARM              = 4;       // CH5  arm switch
const uint8_t  CH_MODE             = 5;       // CH6  manual / Jetson / autonomous

// Deadbands & thresholds
const uint16_t ARM_THRESHOLD       = 1700;    // CH5 above this = armed
const uint16_t MODE_MANUAL_MAX     = 1333;
const uint16_t MODE_JETSON_MAX     = 1666;
const uint16_t MODE_SETTLE_MS      = 100;     // reject transient RC mode changes
const uint16_t STEER_DEADBAND      = 30;      // ±µs around centre
const uint16_t DIR_DEADBAND        = 50;      // ±µs CH2 centre — inside = stop
const uint16_t DIR_ENGAGE          = 250;     // CH2 must exceed this to engage drive
                                              // (prevents turn-stick spillage from
                                              //  accidentally engaging fwd/rev)

// Steering
const float    STEER_SENSITIVITY   = 1.0f;    // 1.0 = full tank-turn effect

// DAC (0 = 0 V, 255 = 3.3 V — tune to your controller's throttle input range)
const uint8_t  DAC_IDLE            = 0;
const uint8_t  DAC_MIN_MOVE        = 150;    // minimum DAC to overcome motor controller dead zone
const uint8_t  DAC_MAX             = 255;

// Throttle ramp (DAC counts per 20 ms tick)
// RAMP_UP = 3  → ~700 ms 0→full  (smooth)
// RAMP_UP = 6  → ~350 ms 0→full  (moderate)
// RAMP_UP = 60 → ~40  ms 0→full  (original — jerky)
// RAMP_DOWN = 255 → instant cut on zero command
const uint8_t  RAMP_UP             = 3;       // smooth acceleration
const uint8_t  RAMP_DOWN           = 255;     // instant stop

// Direction-change sequencer timings (ms)
const uint16_t SEQ_RAMP_BUDGET     = 150;     // max time to reach zero
const uint16_t SEQ_PRE_RELAY       = 150;     // settle before relay flip
const uint16_t SEQ_POST_RELAY      = 300;     // controller accept time after flip (full drive dir changes)
const uint16_t SEQ_POST_RELAY_SHORT= 100;     // shorter settle for low-mag skid-assist flips (<=30% speed)
const uint16_t SEQ_RAMPUP_TIMEOUT  = 1500;    // hard cap on ramp-up phase
const float    SEQ_SHORT_MAG       = 0.30f;   // target speed <= this uses short post settle

// Arming
const uint16_t ARM_DELAY_MS        = 1500;    // power-lock + controller startup
const uint16_t DISARM_RAMP_MS      = 300;     // max time to idle on disarm

// GPIO
const uint8_t  PIN_DAC_L           = 25;      // left  pair throttle
const uint8_t  PIN_DAC_R           = 26;      // right pair throttle
const uint8_t  PIN_REL_L           = 33;      // left  pair reverse relay
const uint8_t  PIN_REL_R           = 32;      // right pair reverse relay
const uint8_t  RELAY_ON            = HIGH;    // change to LOW for active-low modules

// Motor direction — relay OFF = forward on both sides (relay connected to reverse pin)
const bool     L_FWD_RELAY_ON      = false;
const bool     R_FWD_RELAY_ON      = false;

// Serial
const uint32_t BAUD                = 115200;
const uint16_t DEBUG_MS            = 200;
const uint16_t CAN_STATUS_MS       = 200;
const uint16_t CAN_FAULT_MS        = 500;

// =============================================================================
// TYPES
// =============================================================================

enum State   : uint8_t { S_BOOT, S_DISARMED, S_ARMING, S_ARMED, S_DIR_CHG, S_DISARMING, S_FAILSAFE };
enum Dir     : uint8_t { FWD = 0, REV = 1 };
enum ControlMode : uint8_t { MODE_MANUAL, MODE_JETSON, MODE_AUTONOMOUS, MODE_INVALID };
enum SeqPhase: uint8_t { PH_IDLE, PH_RAMP_DOWN, PH_PRE_RELAY, PH_FLIP, PH_RAMP_UP };

struct RxData {
  uint16_t ch[IBUS_CH];
  bool     ok;
  uint32_t lastMs;
};

struct Inputs {
  float steer;      // −1…+1  (CH1)
  float thr;        // 0…1    (CH3)
  bool  arm;        // CH5
  bool  rev;        // CH2 pulled back  → true = reverse
  bool  ch2Active;  // CH2 out of centre deadband → true = fwd/rev drive enabled
  ControlMode mode;
};

struct Side {
  float    req;     // 0–1 requested speed
  float    cur;     // 0–1 current speed (ramped)
  uint8_t  dac;
  Dir      dir;
  Dir      tgtDir;
  SeqPhase phase;
  uint32_t phMs;
};

struct CanCommand {
  int16_t left;
  int16_t right;
  bool enable;
  uint8_t mode;
  uint8_t sequence;
  uint32_t lastMs;
  uint32_t lastSequenceMs;
  uint32_t zeroSinceMs;
  bool sequenceStale;
  bool valid;
};

// =============================================================================
// GLOBALS
// =============================================================================

State   g_state    = S_BOOT,  g_prev = S_BOOT;
RxData  g_rx       = {};
Inputs  g_in       = {};
Side    g_L        = {},      g_R    = {};
bool    g_pendL    = false,   g_pendR = false;
bool    g_failsafe = false;
bool    g_canFailsafe = false;
bool    g_canKillLatched = false;
CanCommand g_can = {};
ControlMode g_prevMode = MODE_INVALID;
ControlMode g_modeCandidate = MODE_INVALID;
uint32_t g_armMs = 0, g_disMs = 0, g_dbgMs = 0, g_loopMs = 0;
uint32_t g_modeCandidateMs = 0;
uint32_t g_canStatusMs = 0, g_canFaultMs = 0, g_canHealthMs = 0;

// =============================================================================
// iBUS PARSER  (UART2, 115200 baud, RX-only)
// =============================================================================
namespace IBus {
  static HardwareSerial _ser(2);
  static uint8_t _buf[32], _idx = 0;

  void begin() { _ser.begin(115200, SERIAL_8N1, IBUS_GPIO, -1); }

  static bool csum() {
    uint16_t s = 0;
    for (uint8_t i = 0; i < 30; i++) s += _buf[i];
    return (0xFFFF - s) == (_buf[30] | (uint16_t)_buf[31] << 8);
  }

  void update(RxData &d) {
    while (_ser.available()) {
      uint8_t b = _ser.read();
      if (_idx == 0 && b != 0x20) continue;
      if (_idx == 1 && b != 0x40) { _idx = 0; continue; }
      _buf[_idx++] = b;
      if (_idx == 32) {
        if (csum()) {
          for (uint8_t c = 0; c < IBUS_CH; c++)
            d.ch[c] = _buf[2+c*2] | (uint16_t)_buf[3+c*2] << 8;
          d.ok = true; d.lastMs = millis();
        }
        _idx = 0;
      }
    }
  }
}

// =============================================================================
// CAN RX (ESP32 built-in TWAI, non-blocking)
// =============================================================================
static bool g_canReady = false;

static int16_t canInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// Forward declaration for recovery use
static void canBegin();
// Defined in the hardware-output section; mode changes may need it sooner.
static void safeAll();

// Recover from BUS-OFF by stopping, uninstalling, and reinitialising TWAI.
// Called automatically when the health check detects BUS-OFF state.
static void canRecover() {
  Serial.println(F("[CAN] BUS-OFF detected — recovering..."));
  g_canReady = false;
  twai_stop();
  twai_driver_uninstall();
  delay(100);
  canBegin();
  Serial.println(g_canReady ? F("[CAN] Recovery OK") : F("[CAN] Recovery FAILED"));
}

// Periodic health check: detect BUS-OFF and recover automatically.
// Also clears a stale TX queue so frames don't back up when Pi is absent.
static void canHealthCheck() {
  if (!g_canReady) return;
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;

  // If BUS-OFF, reinitialise the controller
  if (st.state == TWAI_STATE_BUS_OFF) {
    canRecover();
    return;
  }

  // Drain stale TX queue — prevents queue-full blocking when no peer is present
  if (st.msgs_to_tx > 0) {
    twai_clear_transmit_queue();
  }
}

static void canBegin() {
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  // The ESP32 only consumes control frames.  Reject all other CAN IDs in
  // hardware so unrelated telemetry cannot fill the TWAI receive queue.
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  filter.acceptance_code = CAN_CMD_ID << 21;
  filter.acceptance_mask = ~(0x7FFUL << 21);
  filter.single_filter = true;

  if (twai_driver_install(&general, &timing, &filter) == ESP_OK &&
      twai_start() == ESP_OK) {
    g_canReady = true;
    Serial.println(F("[CAN] TWAI started at 500 kbit/s"));
  } else {
    g_canReady = false;
    Serial.println(F("[CAN] TWAI start failed — check transceiver wiring"));
  }
}

static void canUpdate() {
  if (!g_canReady) return;

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier != CAN_CMD_ID ||
        message.data_length_code != 8 ||
        message.extd || message.rtr ||
        message.data[7] != CAN_MAGIC) {
      continue;
    }

    int16_t left = canInt16(message.data[0], message.data[1]);
    int16_t right = canInt16(message.data[2], message.data[3]);
    if (left < -1000 || left > 1000 || right < -1000 || right > 1000 ||
        (message.data[5] != CAN_MODE_JETSON &&
         message.data[5] != CAN_MODE_AUTONOMOUS)) {
      continue;
    }

    // The mode switch is an independent safety boundary.  Do not let a
    // command intended for Jetson control drive the vehicle while autonomous
    // mode is selected (or vice versa); a mode mismatch must time out instead.
    uint8_t selectedMode = g_prevMode == MODE_JETSON ? CAN_MODE_JETSON :
                           g_prevMode == MODE_AUTONOMOUS ? CAN_MODE_AUTONOMOUS : 0;
    if (message.data[5] != selectedMode) continue;

    uint32_t now = millis();
    bool sequenceChanged = !g_can.valid || message.data[6] != g_can.sequence;
    g_can.left = left;
    g_can.right = right;
    g_can.enable = message.data[4] != 0;
    g_can.mode = message.data[5];
    g_can.sequence = message.data[6];
    g_can.lastMs = now;
    if (sequenceChanged) g_can.lastSequenceMs = now;
    g_can.valid = true;
    g_can.sequenceStale = false;

    // A fresh remote session must prove it is commanding zero before it can
    // arm.  This prevents a queued/nonzero command from moving the vehicle
    // immediately after recovery, a mode switch, or a remote restart.
    bool zeroCommand = abs(left) <= CAN_REARM_ZERO_LIMIT &&
                       abs(right) <= CAN_REARM_ZERO_LIMIT;
    if (!g_can.enable || !zeroCommand) {
      g_can.zeroSinceMs = 0;
    } else if (g_can.zeroSinceMs == 0) {
      g_can.zeroSinceMs = now;
    }
    // A disabled frame is an explicit remote stop.  It remains effective until
    // a subsequent valid, mode-matched enabled frame deliberately re-arms the
    // channel, rather than being confused with a zero-speed command.
    g_canKillLatched = !g_can.enable;
  }
}

static void checkCanFailsafe() {
  // In CAN mode, both frame freshness and sequence progress are required.
  // The enable bit in the frame is the operator's arm signal — no RC gating.
  if (g_prevMode == MODE_MANUAL) {
    g_canFailsafe = false;
    g_canKillLatched = false;
    g_can.sequenceStale = false;
    return;
  }
  uint32_t now = millis();
  bool timedOut = !g_can.valid || (now - g_can.lastMs) > CAN_TIMEOUT_MS;
  g_can.sequenceStale = !timedOut &&
                        (now - g_can.lastSequenceMs) > CAN_SEQUENCE_TIMEOUT_MS;
  g_canFailsafe = timedOut || g_can.sequenceStale;
}

static bool canRearmReady() {
  return g_can.zeroSinceMs != 0 &&
         (millis() - g_can.zeroSinceMs) >= CAN_REARM_ZERO_MS;
}

static void handleModeChange() {
  if (g_in.mode != g_modeCandidate) {
    g_modeCandidate = g_in.mode;
    g_modeCandidateMs = millis();
    return;
  }
  if ((millis() - g_modeCandidateMs) < MODE_SETTLE_MS ||
      g_modeCandidate == g_prevMode) return;

  // Never transfer control live between RC, Jetson, and autonomous sources.
  // The operator/source must explicitly arm again after selecting a mode.
  safeAll();
  g_state = S_DISARMED;
  g_can = {};
  g_canFailsafe = false;
  g_canKillLatched = false;
  g_prevMode = g_modeCandidate;
}

// =============================================================================
// CAN MODULE HEALTH CHECK
// Verifies that the TWAI transceiver wired to D5 (TX) and D4 (RX) is present
// and that the controller is operating normally.
//
// Returns true  — driver running, bus not in bus-off / error-passive state,
//                 and TX/RX error counters below warning threshold.
// Returns false — driver not started, or hardware fault detected.
//
// Prints a detailed report to Serial regardless of outcome.
// Safe to call from setup() after canBegin(), or from a maintenance routine.
// =============================================================================

#define CAN_ERR_WARN_THRESHOLD  96   // IEC 11898 warning level is 96 counts

static bool canVerify() {
  Serial.println(F("[CAN] ---- CAN module self-check ----"));
  Serial.printf ("[CAN]   TX pin : GPIO%d (D5)\n", (int)CAN_TX_PIN);
  Serial.printf ("[CAN]   RX pin : GPIO%d (D4)\n", (int)CAN_RX_PIN);

  // 1. Driver must have been started successfully by canBegin().
  if (!g_canReady) {
    Serial.println(F("[CAN]   FAIL — TWAI driver did not start."));
    Serial.println(F("[CAN]          Check wiring: CANH/CANL, 120 Ω termination,"));
    Serial.println(F("[CAN]          and transceiver VCC/GND."));
    Serial.println(F("[CAN] ---- self-check FAILED ----"));
    return false;
  }

  // 2. Query live controller status from the hardware peripheral.
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println(F("[CAN]   FAIL — could not read TWAI status register."));
    Serial.println(F("[CAN] ---- self-check FAILED ----"));
    return false;
  }

  // 3. Log the raw status fields for diagnostics.
  Serial.printf("[CAN]   State        : ");
  switch (status.state) {
    case TWAI_STATE_STOPPED:       Serial.println(F("STOPPED"));       break;
    case TWAI_STATE_RUNNING:       Serial.println(F("RUNNING ✓"));     break;
    case TWAI_STATE_BUS_OFF:       Serial.println(F("BUS-OFF ✗"));     break;
    case TWAI_STATE_RECOVERING:    Serial.println(F("RECOVERING"));     break;
    default:                       Serial.println(F("UNKNOWN"));        break;
  }
  Serial.printf("[CAN]   TX err count : %lu%s\n",
                (unsigned long)status.tx_error_counter,
                status.tx_error_counter >= CAN_ERR_WARN_THRESHOLD ? "  ← HIGH" : "");
  Serial.printf("[CAN]   RX err count : %lu%s\n",
                (unsigned long)status.rx_error_counter,
                status.rx_error_counter >= CAN_ERR_WARN_THRESHOLD ? "  ← HIGH" : "");
  Serial.printf("[CAN]   TX queued    : %lu\n", (unsigned long)status.msgs_to_tx);
  Serial.printf("[CAN]   RX queued    : %lu\n", (unsigned long)status.msgs_to_rx);
  Serial.printf("[CAN]   Arb lost     : %lu\n", (unsigned long)status.arb_lost_count);
  Serial.printf("[CAN]   Bus errors   : %lu\n", (unsigned long)status.bus_error_count);

  // 4. Evaluate health criteria.
  bool healthy = true;

  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println(F("[CAN]   FAIL — controller is bus-off."));
    Serial.println(F("[CAN]          Likely causes: missing termination resistor,"));
    Serial.println(F("[CAN]          broken CANH/CANL wire, or no other node on bus."));
    healthy = false;
  } else if (status.state != TWAI_STATE_RUNNING) {
    Serial.println(F("[CAN]   WARN — controller is not in RUNNING state."));
    healthy = false;
  }

  if (status.tx_error_counter >= CAN_ERR_WARN_THRESHOLD ||
      status.rx_error_counter >= CAN_ERR_WARN_THRESHOLD) {
    Serial.println(F("[CAN]   WARN — error counter(s) at or above warning threshold."));
    Serial.println(F("[CAN]          Check for missing termination or wiring faults."));
    healthy = false;
  }

  if (status.bus_error_count > 0) {
    Serial.println(F("[CAN]   WARN — bus errors detected since last reset."));
    healthy = false;
  }

  if (healthy) {
    Serial.println(F("[CAN]   OK — CAN module on D5/D4 is connected and operational."));
    Serial.println(F("[CAN] ---- self-check PASSED ----"));
  } else {
    Serial.println(F("[CAN] ---- self-check FAILED ----"));
  }

  return healthy;
}

static bool canSend(uint32_t id, const uint8_t *data) {
  if (!g_canReady) return false;
  twai_message_t message = {};
  message.identifier = id;
  message.data_length_code = 8;
  memcpy(message.data, data, 8);
  return twai_transmit(&message, 0) == ESP_OK;
}

static void canTelemetry() {
  uint32_t now = millis();

  // Run health check every 1 s — recovers from BUS-OFF automatically
  if (now - g_canHealthMs >= 1000) {
    g_canHealthMs = now;
    canHealthCheck();
  }

  if (now - g_canStatusMs >= CAN_STATUS_MS) {
    g_canStatusMs = now;
    uint8_t data[8] = {
      (uint8_t)g_state,
      (uint8_t)g_prevMode,
      g_L.dac,
      g_R.dac,
      (uint8_t)(g_failsafe || g_canFailsafe || g_canKillLatched),
      g_can.sequence,
      (uint8_t)((g_can.valid ? 1 : 0) |
                (g_can.sequenceStale ? 2 : 0) |
                (canRearmReady() ? 4 : 0)),
      CAN_MAGIC
    };
    canSend(CAN_STATUS_ID, data);
  }

  if (now - g_canFaultMs >= CAN_FAULT_MS) {
    g_canFaultMs = now;
    uint8_t data[8] = {
      (uint8_t)(g_canReady ? 0 : 1),
      (uint8_t)(g_canFailsafe ? 1 : 0),
      (uint8_t)(g_failsafe ? 1 : 0),
      (uint8_t)(g_canKillLatched ? 1 : 0),
      (uint8_t)(g_rx.ok ? 0 : 1),
      (uint8_t)(g_can.sequenceStale ? 1 : 0),
      (uint8_t)(canRearmReady() ? 1 : 0),
      CAN_MAGIC
    };
    canSend(CAN_FAULT_ID, data);
  }
}

// =============================================================================
// HELPERS
// =============================================================================

static float clamp(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

static float chanToFloat(uint16_t raw, uint16_t mid, uint16_t db) {
  int16_t off = (int16_t)raw - (int16_t)mid;
  if (abs(off) <= (int16_t)db) return 0.0f;
  float range = (IBUS_MAX - IBUS_MIN) / 2.0f - db;
  return clamp((off > 0 ? 1.0f : -1.0f) * (abs(off) - db) / range, -1.0f, 1.0f);
}

static uint8_t toDac(float s) {
  if (s <= 0.0f) return DAC_IDLE;
  // Remap 0→1 speed to DAC_MIN_MOVE→DAC_MAX so motor controllers always
  // receive enough voltage to overcome their dead zone.
  float mapped = DAC_MIN_MOVE + clamp(s, 0.0f, 1.0f) * (DAC_MAX - DAC_MIN_MOVE);
  return (uint8_t)mapped;
}

static float rampTo(float cur, float tgt, float step) {
  if (cur < tgt) return min(cur+step, tgt);
  if (cur > tgt) return max(cur-step, tgt);
  return tgt;
}

// =============================================================================
// HARDWARE OUTPUT
// =============================================================================

static void wDAC(uint8_t pin, uint8_t v)  {
#ifndef SIMULATION_MODE
  dacWrite(pin, v);
#endif
}

static void wRelay(uint8_t pin, bool on) {
#ifndef SIMULATION_MODE
  digitalWrite(pin, on ? RELAY_ON : (RELAY_ON == HIGH ? LOW : HIGH));
#endif
}

// Resolve logical direction to physical relay state for a given side.
static bool relayFor(bool isLeft, Dir dir) {
  bool fwdOn = isLeft ? L_FWD_RELAY_ON : R_FWD_RELAY_ON;
  return (dir == FWD) ? fwdOn : !fwdOn;
}

static void flushDAC()    { wDAC(PIN_DAC_L, g_L.dac); wDAC(PIN_DAC_R, g_R.dac); }
static void flushRelays() { wRelay(PIN_REL_L, relayFor(true,  g_L.dir));
                            wRelay(PIN_REL_R, relayFor(false, g_R.dir)); }

static void safeAll() {
  g_L = {}; g_R = {};
  g_pendL = g_pendR = false;
  wDAC(PIN_DAC_L, DAC_IDLE); wDAC(PIN_DAC_R, DAC_IDLE);
  wRelay(PIN_REL_L, relayFor(true,  FWD));
  wRelay(PIN_REL_R, relayFor(false, FWD));
}

static void initHW() {
  pinMode(PIN_REL_L,   OUTPUT);
  pinMode(PIN_REL_R,   OUTPUT);
  safeAll();
  IBus::begin();
}

// =============================================================================
// INPUT NORMALISATION
// =============================================================================

static void checkFailsafe() {
  if (g_rx.lastMs == 0) { g_failsafe = false; return; }
  bool lost = (millis() - g_rx.lastMs) > FAILSAFE_MS;
  if (lost != g_failsafe) {
    g_failsafe = lost;
    g_rx.ok = !lost;
    Serial.println(lost ? F("[FS] Lost") : F("[FS] Restored — rearm required"));
  }
}

static void readInputs() {
  IBus::update(g_rx);

  if (!g_rx.ok) { g_in = {}; return; }

  // Clamp all channels to valid iBUS range before use — guards against
  // corrupted frames that pass checksum but contain out-of-range values.
  auto chSafe = [](uint16_t v) -> uint16_t {
    return (v < IBUS_MIN) ? IBUS_MIN : (v > IBUS_MAX) ? IBUS_MAX : v;
  };

  g_in.steer = chanToFloat(chSafe(g_rx.ch[CH_STEER]), IBUS_MID, STEER_DEADBAND);

  float t = (float)(chSafe(g_rx.ch[CH_THR]) - IBUS_MIN) / (IBUS_MAX - IBUS_MIN);
  g_in.thr = (t < 0.02f) ? 0.0f : clamp(t, 0.0f, 1.0f);

  // CH2: three-zone spring-return stick.
  //   Push forward  (> mid + DIR_ENGAGE) → FWD drive enabled
  //   Centre zone   (within DIR_ENGAGE)  → no fwd/rev; tank turn possible via CH1
  //   Pull back     (< mid - DIR_ENGAGE) → REV drive enabled
  // Throttle (CH3) is never touched here — it is always the speed source.
  int16_t d = (int16_t)chSafe(g_rx.ch[CH_DIR]) - IBUS_MID;
  if      (d >  (int16_t)DIR_ENGAGE) { g_in.rev = false; g_in.ch2Active = true;  }
  else if (d < -(int16_t)DIR_ENGAGE) { g_in.rev = true;  g_in.ch2Active = true;  }
  else                               { g_in.rev = false;  g_in.ch2Active = false; }

  g_in.arm = (chSafe(g_rx.ch[CH_ARM]) > ARM_THRESHOLD);

  uint16_t modeRaw = chSafe(g_rx.ch[CH_MODE]);
  if (modeRaw <= MODE_MANUAL_MAX) {
    g_in.mode = MODE_MANUAL;
  } else if (modeRaw <= MODE_JETSON_MAX) {
    g_in.mode = MODE_JETSON;
  } else {
    g_in.mode = MODE_AUTONOMOUS;
  }
}

// =============================================================================
// STEERING MIX
// =============================================================================

static void flagDir(Side &s, bool &pend, Dir want) {
  if (want != s.dir) {
    s.tgtDir = want;
    if (s.phase == PH_IDLE && !pend) pend = true;
  } else if (s.phase == PH_IDLE) {
    s.tgtDir = want;
  }
}

static void mix() {
  float str = g_in.steer * STEER_SENSITIVITY;  // −1 = left, +1 = right  (CH1)
  float thr = g_in.thr;                         // 0–1  from CH3 slider
  float mag = fabsf(str);

  // --------------------------------------------------------------------------
  // Three drive modes determined by CH2 position:
  //
  // CH2 forward  (rev=false, ch2Active=true)  → forward drive + skid steer
  // CH2 reverse  (rev=true,  ch2Active=true)  → reverse drive + skid steer
  // CH2 centre   (ch2Active=false)            → tank turn in place if CH1 active
  //                                             and CH3 > 0, otherwise stop
  //
  // CH3 (throttle slider) is ALWAYS the speed source — nothing moves without it.
  // --------------------------------------------------------------------------

  if (thr < 0.01f) {
    // No throttle → full stop regardless of any other stick
    g_L.req = g_R.req = 0.0f;
    flagDir(g_L, g_pendL, FWD);
    flagDir(g_R, g_pendR, FWD);
    return;
  }

  if (!g_in.ch2Active) {
    // CH2 centred → tank turn mode.
    // CH1 left  (str < 0): left=REV, right=FWD → spins left
    // CH1 right (str > 0): left=FWD, right=REV → spins right
    if (str < -0.02f) {
      g_L.req = thr;  g_R.req = thr;
      flagDir(g_L, g_pendL, REV);
      flagDir(g_R, g_pendR, FWD);
    } else if (str > 0.02f) {
      g_L.req = thr;  g_R.req = thr;
      flagDir(g_L, g_pendL, FWD);
      flagDir(g_R, g_pendR, REV);
    } else {
      g_L.req = g_R.req = 0.0f;
      flagDir(g_L, g_pendL, FWD);
      flagDir(g_R, g_pendR, FWD);
    }
    return;
  }

  // CH2 forward or reverse → skid steer drive.
  Dir   baseDir = g_in.rev ? REV : FWD;
  float inner   = clamp(thr * (1.0f - mag), 0.0f, 1.0f);

  if (str < -0.02f) {
    // Turn LEFT while driving → right is inner (slower), left is outer (faster)
    g_L.req = thr;
    g_R.req = inner;
  } else if (str > 0.02f) {
    // Turn RIGHT while driving → left is inner (slower), right is outer (faster)
    g_L.req = inner;
    g_R.req = thr;
  } else {
    // Straight
    g_L.req = g_R.req = thr;
  }
  flagDir(g_L, g_pendL, baseDir);
  flagDir(g_R, g_pendR, baseDir);
}

static void mixCan() {
  // Drive purely from CAN enable bit — no RC arm required in CAN mode.
  // enable=0, invalid sequence progress, or no recent frame → stop safely.
  if (!g_can.enable || g_canKillLatched || g_canFailsafe) {
    g_L.req = g_R.req = 0.0f;
    flagDir(g_L, g_pendL, FWD);
    flagDir(g_R, g_pendR, FWD);
    return;
  }

  // Direct left/right mapping — no swap needed.
  float left  = (float)g_can.left  / 1000.0f;
  float right = (float)g_can.right / 1000.0f;

  // Small deadband — ignore tiny values that would cause spurious relay switching
  if (fabsf(left)  < 0.05f) left  = 0.0f;
  if (fabsf(right) < 0.05f) right = 0.0f;

  g_L.req = fabsf(left);
  g_R.req = fabsf(right);
  flagDir(g_L, g_pendL, left  < 0.0f ? REV : FWD);
  flagDir(g_R, g_pendR, right < 0.0f ? REV : FWD);
}

// =============================================================================
// THROTTLE RAMP
// =============================================================================

static void rampSide(Side &s, float tgt) {
  float up = (float)RAMP_UP / (DAC_MAX - DAC_IDLE);
  float dn = (float)RAMP_DOWN / (DAC_MAX - DAC_IDLE);
  bool brakingToZero = (tgt <= 0.0f && tgt < s.cur);
  float step = brakingToZero ? dn : up;
  s.cur = rampTo(s.cur, tgt, step);
  s.dac = toDac(s.cur);
}

static bool rampIdle(Side &s) { rampSide(s, 0.0f); return s.cur <= 0.0f; }

// =============================================================================
// DIRECTION-CHANGE SEQUENCER  (per-side, non-blocking)
// IDLE → RAMP_DOWN → PRE_RELAY → FLIP → RAMP_UP → IDLE
// =============================================================================

static bool runSeq(Side &s, uint8_t pin, bool isLeft, bool &pend) {
  uint32_t now = millis();

  switch (s.phase) {
    case PH_IDLE:
      if (!pend) return true;
      s.phase = PH_RAMP_DOWN; s.phMs = now; pend = false;
      break;

    case PH_RAMP_DOWN:
      if (s.cur <= 0.01f) {
        s.cur = 0.0f; s.dac = DAC_IDLE;
        s.phase = PH_FLIP; s.phMs = 0;
      } else if (rampIdle(s) || (now - s.phMs) > SEQ_RAMP_BUDGET) {
        s.cur = 0.0f; s.dac = DAC_IDLE;
        s.phase = PH_PRE_RELAY; s.phMs = now;
      }
      break;

    case PH_PRE_RELAY:
      if ((now - s.phMs) >= SEQ_PRE_RELAY) { s.phase = PH_FLIP; s.phMs = 0; }
      break;

    case PH_FLIP: {
      if (s.phMs == 0) {
        wRelay(pin, relayFor(isLeft, s.tgtDir));
        s.dir = s.tgtDir; s.phMs = now;
      }
      uint32_t post = (s.req <= SEQ_SHORT_MAG) ? SEQ_POST_RELAY_SHORT : SEQ_POST_RELAY;
      if ((now - s.phMs) >= post) { s.phase = PH_RAMP_UP; s.phMs = now; }
      break;
    }

    case PH_RAMP_UP:
      rampSide(s, s.req);
      if (fabsf(s.cur - s.req) < 0.01f ||
          (now - s.phMs) > SEQ_RAMPUP_TIMEOUT ||
          s.req <= 0.0f) {
        s.phase = PH_IDLE; return true;
      }
      break;

    default: s.phase = PH_IDLE; return true;
  }
  return false;
}

// =============================================================================
// FSM STATE HANDLERS
// =============================================================================

static void onDisarmed() {
  // CAN mode: auto-arm when Pi sends enable=1 — no RC arm needed.
  if (g_prevMode != MODE_MANUAL && g_can.valid && g_can.enable &&
      !g_canKillLatched && !g_canFailsafe && canRearmReady() &&
      (millis() - g_can.lastMs) < CAN_TIMEOUT_MS) {
    Serial.println(F("[FSM] DISARMED → ARMED (CAN enable)"));
    g_state = S_ARMED;
    return;
  }
  // Manual mode: RC arm switch as normal.
  if (g_prevMode == MODE_MANUAL && g_in.arm) {
    Serial.println(F("[FSM] DISARMED → ARMING"));
    g_armMs = millis(); g_state = S_ARMING;
  }
}

static void onArming() {
  if (!g_in.arm) {
    Serial.println(F("[FSM] ARMING aborted → DISARMED"));
    safeAll(); g_state = S_DISARMED;
    return;
  }
  if ((millis() - g_armMs) >= ARM_DELAY_MS) {
    Serial.println(F("[FSM] ARMED")); g_state = S_ARMED;
  }
}

static void onArmed() {
  // In CAN mode: RC arm switch is not required — CAN enable bit is the operator control.
  // In manual mode: RC arm switch disarms as normal.
  if (g_prevMode == MODE_MANUAL && !g_in.arm) {
    Serial.println(F("[FSM] ARMED → DISARMING"));
    g_disMs = millis(); g_state = S_DISARMING; return;
  }

  if (g_prevMode == MODE_MANUAL) {
    mix();
  } else {
    mixCan();
  }

  if (g_pendL || g_pendR || g_L.phase != PH_IDLE || g_R.phase != PH_IDLE) {
    g_state = S_DIR_CHG; return;
  }

  if (g_L.req <= 0.0f && g_R.req <= 0.0f) {
    g_L.cur = g_R.cur = 0.0f;
    g_L.dac = g_R.dac = DAC_IDLE;
  } else {
    rampSide(g_L, g_L.req);
    rampSide(g_R, g_R.req);
  }
  flushDAC(); flushRelays();
}

static void onDirChg() {
  // Do NOT call mix() here — keep the direction targets locked for the full
  // sequence. Calling mix() mid-sequence would overwrite req/tgtDir while
  // relays are switching, causing one side to fire before the other is ready.
  bool ld = runSeq(g_L, PIN_REL_L, true,  g_pendL);
  bool rd = runSeq(g_R, PIN_REL_R, false, g_pendR);

  // Only output DAC once BOTH sides have finished their sequences so motors
  // start together. While waiting, hold DAC at idle.
  if (ld && rd) {
    rampSide(g_L, g_L.req);
    rampSide(g_R, g_R.req);
    flushDAC(); flushRelays();
    g_state = S_ARMED;
  } else {
    // Keep motors off during relay switching
    wDAC(PIN_DAC_L, DAC_IDLE);
    wDAC(PIN_DAC_R, DAC_IDLE);
    flushRelays();
  }
}

static void onDisarming() {
  rampIdle(g_L); rampIdle(g_R); flushDAC();
  if ((g_L.cur <= 0.0f && g_R.cur <= 0.0f) || (millis()-g_disMs) > DISARM_RAMP_MS) {
    safeAll();
    Serial.println(F("[FSM] DISARMED")); g_state = S_DISARMED;
  }
}

static void onFailsafe() {
  // Exit failsafe when the underlying cause clears.
  // RC failsafe: require disarm before re-entry (operator must acknowledge).
  // CAN failsafe/stop: require a fresh, valid enabled frame before re-entry.
  if (g_prevMode == MODE_MANUAL) {
    if (g_failsafe) return;  // RC signal still lost — stay in failsafe
    if (!g_in.arm) {
      Serial.println(F("[FSM] FAILSAFE → DISARMED (RC acknowledged)"));
      g_state = S_DISARMED;
    }
    return;
  }

  if (!g_canFailsafe && !g_canKillLatched) {
    // CAN channel has resumed and was explicitly re-armed.  Return through
    // DISARMED so the normal arming path remains the sole way to drive.
    Serial.println(F("[FSM] FAILSAFE → DISARMED (CAN re-armed)"));
    g_state = S_DISARMED;
  }
}

// =============================================================================
// FSM DISPATCHER
// =============================================================================

static void runFSM() {
  if (g_state != g_prev) {
    if (g_state == S_DISARMED || g_state == S_FAILSAFE) {
      safeAll();
    }
    g_prev = g_state;
  }

  switch (g_state) {
    case S_BOOT:     safeAll(); g_state = S_DISARMED; break;
    case S_DISARMED: onDisarmed();  break;
    case S_ARMING:   onArming();    break;
    case S_ARMED:    onArmed();     break;
    case S_DIR_CHG:  onDirChg();    break;
    case S_DISARMING:onDisarming(); break;
    case S_FAILSAFE: onFailsafe();  break;
    default:         g_state = S_FAILSAFE; break;
  }
}

static void applyFailsafe() {
  // RC failsafe only applies in manual mode.
  // In CAN mode, a command timeout or explicit remote stop is a failsafe.
  if (g_prevMode == MODE_MANUAL) {
    if (g_in.arm && g_failsafe && g_state != S_FAILSAFE) {
      g_state = S_FAILSAFE;
    }
  } else {
    // A timeout or explicit disabled frame must interrupt every active state,
    // including a relay direction-change sequence.
    if ((g_canFailsafe || g_canKillLatched) &&
        g_state != S_DISARMED && g_state != S_FAILSAFE) {
      Serial.println(g_canFailsafe ? F("[FSM] CAN timeout → FAILSAFE") :
                                      F("[FSM] CAN stop → FAILSAFE"));
      g_state = S_FAILSAFE;
    }
  }
}

// =============================================================================
// DEBUG OUTPUT
// =============================================================================

static const char* sName(State s) {
  switch(s) {
    case S_BOOT:     return "BOOT";
    case S_DISARMED: return "DISARMED";
    case S_ARMING:   return "ARMING";
    case S_ARMED:    return "ARMED";
    case S_DIR_CHG:  return "DIR_CHG";
    case S_DISARMING:return "DISARMING";
    case S_FAILSAFE: return "FAILSAFE";
    default:         return "?";
  }
}

static const char* phName(SeqPhase p) {
  switch(p) {
    case PH_IDLE:      return "IDLE";
    case PH_RAMP_DOWN: return "RAMP_DN";
    case PH_PRE_RELAY: return "PRE_REL";
    case PH_FLIP:      return "FLIP";
    case PH_RAMP_UP:   return "RAMP_UP";
    default:           return "?";
  }
}

static const char* modeName(ControlMode m) {
  switch(m) {
    case MODE_MANUAL:     return "MANUAL";
    case MODE_JETSON:     return "JETSON";
    case MODE_AUTONOMOUS: return "AUTO";
    default:              return "INVALID";
  }
}

static void dbg() {
  if ((millis() - g_dbgMs) < DEBUG_MS) return;
  g_dbgMs = millis();
  Serial.println(F("--------------------------------------------------"));
  Serial.printf("UGV v" FW_VERSION "  State : %s\n", sName(g_state));
  Serial.printf("RX     : %s  (%lu ms ago)\n", g_rx.ok?"OK":"LOST", millis()-g_rx.lastMs);
  Serial.printf("Mode   : %s  arm=%d\n", modeName(g_prevMode), g_in.arm);
  Serial.printf("Inputs : str=%+.2f  thr=%.2f  rev=%d  ch2=%s\n",
                g_in.steer, g_in.thr, g_in.rev,
                g_in.ch2Active ? (g_in.rev?"REV":"FWD") : "CTR");
  Serial.printf("CAN    : valid=%d  enable=%d  L=%d  R=%d  fs=%s  seq=%s  rearm=%s\n",
                g_can.valid, g_can.enable, g_can.left, g_can.right,
                g_canFailsafe?"ACTIVE":"clear",
                g_can.sequenceStale?"STALE":"ok",
                canRearmReady()?"ready":"zero-hold");
  Serial.printf("LEFT   : %s  req=%.2f  cur=%.2f  dac=%u  ph=%s\n",
                g_L.dir==REV?"REV":"FWD", g_L.req, g_L.cur, g_L.dac, phName(g_L.phase));
  Serial.printf("RIGHT  : %s  req=%.2f  cur=%.2f  dac=%u  ph=%s\n",
                g_R.dir==REV?"REV":"FWD", g_R.req, g_R.cur, g_R.dac, phName(g_R.phase));
  Serial.printf("Relays : L=%s  R=%s  FS=%s\n",
                relayFor(true, g_L.dir) ?"ON":"OFF",
                relayFor(false,g_R.dir) ?"ON":"OFF",
                g_failsafe?"ACTIVE":"clear");
}

// =============================================================================
// ENTRY POINTS
// =============================================================================

void setup() {
#if ESP_IDF_VERSION_MAJOR >= 5
  const esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  Serial.begin(BAUD); delay(200);
  Serial.println(F("ESP32 UGV v" FW_VERSION));
#ifdef SIMULATION_MODE
  Serial.println(F("*** SIMULATION — no hardware outputs ***"));
#endif

  initHW();
  canBegin();
  canVerify();        // report CAN module health on D5/D4 at startup
  g_loopMs = millis();
}

void loop() {
  // Stable 50 Hz rate — subtract elapsed to prevent drift accumulation.
  uint32_t now = millis();
  if ((now - g_loopMs) < LOOP_MS) return;
  g_loopMs += LOOP_MS;  // advance by fixed period, not by "now"

  esp_task_wdt_reset();  // pat the watchdog — proves we're not stuck

  readInputs();
  checkFailsafe();
  handleModeChange();
  canUpdate();
  checkCanFailsafe();
  applyFailsafe();
  runFSM();
  canTelemetry();
  dbg();
}
