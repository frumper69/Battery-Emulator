#pragma once

#include "UdsCanBattery.h"

class Mg4Battery : public UdsCanBattery {
 public:
  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual uint16_t handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  virtual uint32_t calculate_max_discharge_power_W();
  virtual uint32_t calculate_max_charge_power_W();
  //virtual uint32_t calculate_pack_voltage_limit_max_dV();

  static constexpr const char* Name = "MG4 battery";

  String get_uds_info_html() override;
  const char* get_dtc_json_filename() override { return "mg_dtc.json"; }

 private:
  // (Re)computes max/min_cell_voltage_mV, the working cell-voltage window,
  // and max/min_design_voltage_dV from the current chemistry, number_of_cells
  // and user overrides. Called from setup(), and again whenever chemistry or
  // cell count auto-detection resolves to something new at runtime.
  void apply_cell_voltage_limits();
  static const uint16_t MAX_CELL_DEVIATION_MV = 150;

  // --- Chemistry / cell-count auto-detection -------------------------------
  // Known MG4 pack variants seen in the wild (2026-09): 49kWh LFP (100s),
  // 51kWh LFP (104s), 64kWh NMC (104s), 77kWh NMC (108s). Cell count and
  // chemistry vary independently of each other, so each is detected
  // separately from real telemetry rather than assumed from one another.
  //
  // Chemistry: settled pack voltage clusters cleanly by chemistry regardless
  // of series count (~337-355V observed for LFP variants, ~380-395V for NMC
  // variants) - a threshold roughly in the middle of that ~25V gap separates
  // them with plenty of margin. Only used when the user explicitly selects
  // Autodetect; otherwise their selection is trusted as-is.
  static const uint16_t CHEMISTRY_AUTODETECT_THRESHOLD_DV = 3650;  // 365.0V
  bool chemistry_autodetected = false;

  // Cell count: 0x159 addr 0x510 carries cells 81+ in groups of 4, muxed by
  // its last payload byte. The highest mux value actually seen tells us how
  // many cells beyond the first 80 (from addr 0x509) this pack really has,
  // regardless of which of the known (or a future) variant it is.
  //
  // On power-up this mux value ramps 1,2,3,...,max once before settling and
  // holding there - it does not cycle back down. That means any "commit
  // after N repeats of the current value" scheme is unsound: an intermediate
  // rung of the ramp (e.g. mux=1, 84 cells) can just as easily sit still for
  // N repeats as the true final value can, if the ramp happens to pace out
  // slowly. Instead, only commit once the highest mux value seen has gone
  // CELL_COUNT_SETTLE_MS without being exceeded - i.e. wait for the ramp to
  // visibly stop climbing, not for any one value to repeat. Also reject
  // implausible mux values outright (0x159's subframes carry no CRC of
  // their own) since a corrupted byte could otherwise masquerade as "the
  // ramp settled high".
  static const uint8_t MAX_PLAUSIBLE_MUX_0x510 = 10;  // 80+10*4=120 cells, headroom past every known variant
  static const unsigned long CELL_COUNT_SETTLE_MS = 1000;
  uint8_t highest_mux_0x510_seen = 0;
  unsigned long last_mux_0x510_increase_millis = 0;
  bool cell_count_confirmed = false;  // logged once, whether or not it changed anything

  int32_t working_cell_min_mV = 0;
  int32_t working_cell_recharge_threshold_mV = 0;
  int32_t working_cell_max_mV = 0;
  // Latched flags for cell-voltage hysteresis (persist between update_values calls)
  bool voltageAtCellMax = false;
  bool voltageAtCellMin = false;
  int32_t cell_voltage_freshness = 0;
  int32_t soc_freshness = 0;
  int32_t temp_freshness = 0;

  int16_t module_temperatures_dC[12] = {0};
  int16_t module_temps_received = 0;

  // For monitoring the actual battery-reported contactor state.
  struct PackContactorFeedback {
    bool received = false;
    uint8_t state = 0xFF;  // 0xFF = no data yet
    bool isClosed() const { return received && state == 7; }
    bool isPrecharging() const { return received && state == 11; }
    // Value for datalayer.system.status.contactors_engaged (shown on the main
    // BE contactor widget): 1=closed, 3=precharge active, 0=otherwise.
    uint8_t contactsEngaged() const {
      if (isClosed()) {
        return 1;
      }
      if (isPrecharging()) {
        return 3;
      }
      return 0;
    }
    const char* label() const {
      if (!received) {
        return "No data received yet";
      }
      switch (state) {
        case 7:
          return "Closed / charging";
        case 11:
          return "Precharge active";
        case 3:
          return "Idle";
        default:
          return "Unknown";
      }
    }
    const char* color() const {
      if (!received) {
        return "#9e9e9e";  // Grey
      }
      switch (state) {
        case 7:
          return "#4CAF50";  // Green
        case 11:
          return "#ff9800";  // Orange
        case 3:
          return "#f44336";  // Red
        default:
          return "#9e9e9e";  // Grey
      }
    }
  };

  // Contactor management state machine.
  enum class ContactorState {
    WAITING_FOR_PACK,  // Silent: waiting for the first 0x15B state (or grace expiry)
    CLOSING,           // Replaying the full message cycle from index 0
    CLOSED,            // Pack confirmed closed, replaying the end of the cycle
    OPENING,           // Open requested, replaying the start of the cycle
  };

  bool reportsFDVoltages = false;
  bool reportsSoC = false;
  bool coulombCounting = false;
  ContactorState contactorState = ContactorState::WAITING_FOR_PACK;
  PackContactorFeedback pack_contactors;
  unsigned long contactorWaitStartMillis = 0;  // Grace timer base while WAITING_FOR_PACK
  int replayFrameIndex047_08A = 0;             // Master cursor through the message cycle; the
                                               // 313/314/315 index is derived from it (see cpp)
  int wakeupCounter = 0;                       // Paces the 0x4F3 FD wakeup keep-alive

  void contactor_state_tick(unsigned long currentMillis);

  uint32_t total_discharge_dC = 0;  // in deci-Coulombs
  bool total_discharge_initialized = false;

  unsigned long lastTickMillis = 0;

  unsigned long previousMillis10 = 0;   // will store last time a 10ms CAN Message was send
  unsigned long previousMillis200 = 0;  // will store last time a 200ms CAN Message was send

  uint32_t* nonvolatile_cookie = 0;
  uint32_t* nonvolatile_total_discharge_dC = 0;
  static const uint32_t NONVOLATILE_COOKIE_VALUE = 0x7734b1f5;

  static const uint16_t POLL_BATTERY_VOLTAGE = 0xB042;
  static const uint16_t POLL_BATTERY_CURRENT = 0xB043;
  static const uint16_t POLL_BATTERY_SOC = 0xB046;
  static const uint16_t POLL_MIN_CELL_TEMPERATURE = 0xB057;
  static const uint16_t POLL_MAX_CELL_TEMPERATURE = 0xB056;
  static const uint16_t POLL_BATTERY_SOH = 0xB061;

  CAN_frame MG4_4F3_FD = {.FD = true,
                          .ext_ID = false,
                          .DLC = 8,
                          .ID = 0x4F3,
                          .data = {0xF3, 0x10, 0x48, 0x00, 0xFF, 0xFF, 0x00, 0x11}};
  // 0x047 (FD) and 0x08A are populated at runtime by concise generators (see
  // MG-4-BATTERY.cpp) rather than by replaying a long captured table.
  // (0x313/0x314/0x315 used to be generated the same way, but further
  // testing found the pack doesn't need them, so they're no longer built or
  // sent.)
  CAN_frame MG4_047_FD = {.FD = true,
                          .ext_ID = false,
                          .DLC = 24,
                          .ID = 0x047,
                          .data = {0x00, 0x01, 0x27, 0x08, 0xF4, 0xF0, 0x80, 0x04, 0x00, 0x5F, 0x3C, 0x00,
                                   0x00, 0x01, 0x48, 0x08, 0x61, 0xF0, 0x6A, 0x06, 0xA0, 0xFF, 0xF0, 0xFF}};
  CAN_frame MG4_08A_FD = {.FD = true, .ext_ID = false, .DLC = 48, .ID = 0x08A, .data = {0}};
};