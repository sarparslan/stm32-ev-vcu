
#ifndef EV_CONFIG_H
#define EV_CONFIG_H

/*
 * Safety thresholds.
 */
#define BATTERY_TEMP_FAULT_THRESHOLD_C       80     // signed: temps can be negative
#define MOTOR_TEMP_FAULT_THRESHOLD_C        120     // motor windings tolerate more than cells
#define BATTERY_UNDER_VOLTAGE_THRESHOLD_dV   440U   // 44.0 V
#define BATTERY_OVER_VOLTAGE_THRESHOLD_dV    580U   // 58.0 V
#define BATTERY_SOC_FULL_PERCENT            95U

/*
 * Charging command defaults.
 */
#define DEFAULT_CHARGE_TARGET_VOLTAGE_dV     560U   // 56.0 V
#define DEFAULT_CHARGE_TARGET_CURRENT_dA     100U   // 10.0 A

/*
 * LED diagnostics timing.
 */
#define FAULT_LED_BLINK_PERIOD_MS           500U

/*
 * CAN communication timing.
 */
#define CAN_NODE_TIMEOUT_MS                 1000U   // node silent this long -> FAULT
#define CAN_TX_PERIOD_MS                    100U    // VCU command frame period

/*
 * How often CAN frames are dumped to the debug console (UART2).
 */
#define CAN_TRACE_PERIOD_MS                 500U

/*
 * Signed value encoding on CAN.
 *
 * CAN data bytes are unsigned, so negative-capable values are
 * transmitted with a fixed offset baked directly into the
 * decode/encode lines (temp +40, current +32000, rpm +32000):
 *
 *   wire (unsigned) = real_value + offset
 *   real_value      = (signed)wire - offset
 */

/*
 * Incoming status frames.
 *
 * Each node publishes its status over 3 separate CAN frames.
 * The byte layout of every frame is documented next to its
 * decoder in the matching *_can.c file.
 */

/* BMS status frames */
#define CAN_ID_BMS_STATUS_1                 0x180U   // voltage + current
#define CAN_ID_BMS_STATUS_2                 0x181U   // temp + soc + state
#define CAN_ID_BMS_STATUS_3                 0x182U   // packed flags

/* Motor status frames */
#define CAN_ID_MOTOR_STATUS_1               0x190U   // rpm + current
#define CAN_ID_MOTOR_STATUS_2               0x191U   // temp + state
#define CAN_ID_MOTOR_STATUS_3               0x192U   // flags + alive

/* Charger status frames */
#define CAN_ID_CHARGER_STATUS_1             0x1A0U   // voltage + current
#define CAN_ID_CHARGER_STATUS_2             0x1A1U   // state + plug
#define CAN_ID_CHARGER_STATUS_3             0x1A2U   // flags + alive

/*
 * Outgoing ECU command frames.
 */
#define CAN_ID_BMS_COMMAND                  0x280U
#define CAN_ID_MOTOR_COMMAND                0x281U
#define CAN_ID_CHARGER_COMMAND              0x282U

#endif /* EV_CONFIG_H */
