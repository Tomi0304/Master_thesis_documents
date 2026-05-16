#!/usr/bin/env python3
"""
test_01_latency_check.py

Verify that the 200 Hz control loop holds its timing AND characterise
the effective report rate of the BNO085 IMU on the Raspberry Pi.

Important:
    This version does NOT estimate the IMU rate from quat != last_quat.
    Instead, it counts the actual BNO085 GAME_ROTATION_VECTOR reports
    processed by the Adafruit driver.

Usage:
    sudo pigpiod
    python3 test01.py
"""

import time
import numpy as np

from trms_utils import APHardware, CSVLogger, LOOP_DT, precise_sleep_until


# =============================================================================
# TEST PARAMETERS
# =============================================================================

DURATION_S = 60.0

I2C_FREQ_HZ = 400_000
REPORT_INTERVAL_US = 5_000  # 200 Hz requested to the BNO085

WARMUP_SAMPLES = 10


# =============================================================================
# IMU INTERFACE
# =============================================================================

class BNO085Reader:
    """
    BNO085 reader for bandwidth characterisation.

    Main metric:
        imu_reports

    This counts the actual GAME_ROTATION_VECTOR reports processed by the
    Adafruit driver. This is much better than checking whether the quaternion
    value changed, because when the IMU is nearly static, several consecutive
    quaternions can be numerically identical.
    """

    def __init__(self):
        import board
        import busio
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR

        self.BNO_REPORT_GAME_ROTATION_VECTOR = BNO_REPORT_GAME_ROTATION_VECTOR

        # Public counters
        self.report_count = 0
        self.value_change_count = 0
        self.report_timestamps = []

        # Internal state
        self._last_quat = None
        self._hook_installed = False

        # I2C init
        #
        # On Raspberry Pi/Linux, Blinka may print:
        # "I2C frequency is not settable in python, ignoring!"
        #
        # This is normal if the speed is already set in:
        # /boot/firmware/config.txt
        #
        #     dtparam=i2c_arm_baudrate=400000
        #
        i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ_HZ)
        self.bno = BNO08X_I2C(i2c)

        # Install the report counter before enabling the feature
        self._install_report_counter_hook()

        # Request 200 Hz GAME_ROTATION_VECTOR
        self.bno.enable_feature(
            BNO_REPORT_GAME_ROTATION_VECTOR,
            REPORT_INTERVAL_US
        )

        # Let sensor and driver settle
        time.sleep(0.5)

        # Discard startup/enable reports
        self.report_count = 0
        self.value_change_count = 0
        self.report_timestamps.clear()
        self._last_quat = None

    def _install_report_counter_hook(self):
        """
        Hook the internal Adafruit driver report processor.

        This is a private API, so it is meant for characterisation/testing,
        not necessarily for final production control code.
        """

        if not hasattr(self.bno, "_process_report"):
            self._hook_installed = False
            return

        original_process_report = self.bno._process_report
        target_report_id = self.BNO_REPORT_GAME_ROTATION_VECTOR
        parent = self

        def counted_process_report(*args, **kwargs):
            """
            Compatible wrapper around _process_report(report_id, report_bytes).

            Using *args makes the hook a bit more robust if the library changes
            slightly in the future.
            """

            if len(args) >= 1:
                report_id = args[0]

                if report_id == target_report_id:
                    parent.report_count += 1
                    parent.report_timestamps.append(time.perf_counter())

            return original_process_report(*args, **kwargs)

        self.bno._process_report = counted_process_report
        self._hook_installed = True

    def try_read(self):
        """
        Poll the IMU once.

        Returns:
            new_reports: number of actual BNO085 reports processed during this poll
            value_changed: 1 if quaternion value changed, 0 otherwise
            read_time_ms: time spent inside the IMU read
            quat: latest quaternion tuple or None
        """

        t0 = time.perf_counter()

        reports_before = self.report_count

        try:
            # This triggers the Adafruit driver to process all available packets
            quat = self.bno.game_quaternion  # (i, j, k, real)

            read_time_ms = (time.perf_counter() - t0) * 1000.0

            new_reports = self.report_count - reports_before

            if quat != self._last_quat:
                value_changed = 1
                self.value_change_count += 1
            else:
                value_changed = 0

            self._last_quat = quat

            return new_reports, value_changed, read_time_ms, quat

        except Exception:
            read_time_ms = (time.perf_counter() - t0) * 1000.0
            return 0, 0, read_time_ms, None

    def report_rate_from_timestamps(self):
        """
        Estimate IMU report rate from timestamps of processed reports.
        This is independent from the loop duration.
        """

        if len(self.report_timestamps) < 2:
            return 0.0

        duration = self.report_timestamps[-1] - self.report_timestamps[0]

        if duration <= 0:
            return 0.0

        return (len(self.report_timestamps) - 1) / duration


# =============================================================================
# MAIN TEST
# =============================================================================

def main():
    target_hz = 1.0 / LOOP_DT

    print("[TEST 01+IMU] Loop timing + IMU report-rate characterisation")
    print(f"Duration: {DURATION_S:.1f} s")
    print(f"Loop target: {target_hz:.0f} Hz")
    print(f"BNO085 requested report interval: {REPORT_INTERVAL_US} us")
    print(f"BNO085 requested report rate: {1e6 / REPORT_INTERVAL_US:.1f} Hz")

    hw = None
    logger = None

    try:
        hw = APHardware()

        try:
            imu = BNO085Reader()
            print("[IMU] BNO085 initialised.")
            print(f"[IMU] Report counter hook installed: {imu._hook_installed}")
        except Exception as e:
            print(f"[IMU] ERROR initialising BNO085: {e}")
            print("Continuing without IMU (loop timing only).")
            imu = None

        logger = CSVLogger(
            "test01_latency_imu_report_count",
            [
                "t_s",
                "dt_ms",
                "imu_reports",
                "imu_value_changed",
                "imu_read_ms",
                "angle_enc_deg",
                "quat_i",
                "quat_j",
                "quat_k",
                "quat_real",
            ],
        )

        print("\nStarting test.")
        print("For this corrected test, you do NOT need to move the arm continuously.")
        print("The main metric is now 'imu_reports', not 'imu_value_changed'.")

        dts = []
        imu_report_counts = []
        imu_value_changes = []
        imu_read_times = []

        next_tick = time.perf_counter() + LOOP_DT
        t_start = time.perf_counter()
        t_prev = t_start

        while time.perf_counter() - t_start < DURATION_S:
            precise_sleep_until(next_tick)

            t_now = time.perf_counter()
            dt_ms = (t_now - t_prev) * 1000.0
            dts.append(dt_ms)

            angle_enc = hw.get_angle_deg()

            if imu is not None:
                new_reports, value_changed, read_ms, quat = imu.try_read()

                imu_report_counts.append(new_reports)
                imu_value_changes.append(value_changed)
                imu_read_times.append(read_ms)

                if quat is not None:
                    qi, qj, qk, qr = quat
                else:
                    qi, qj, qk, qr = np.nan, np.nan, np.nan, np.nan

            else:
                new_reports = 0
                value_changed = 0
                read_ms = 0.0
                qi, qj, qk, qr = np.nan, np.nan, np.nan, np.nan

            logger.log(
                dt_ms,
                new_reports,
                value_changed,
                read_ms,
                angle_enc,
                qi,
                qj,
                qk,
                qr,
            )

            t_prev = t_now
            next_tick += LOOP_DT

        logger.close()
        logger = None

        hw.close()
        hw = None

        # =====================================================================
        # ANALYSIS
        # =====================================================================

        dts = np.array(dts[WARMUP_SAMPLES:])

        print(f"\n{'=' * 60}")
        print("LOOP TIMING")
        print(f"{'=' * 60}")

        if len(dts) > 0:
            measured_duration_s = dts.sum() / 1000.0

            print(f"  Iterations analysed:       {len(dts)}")
            print(f"  Target dt:                 {LOOP_DT * 1000:.3f} ms")
            print(f"  Mean dt:                   {dts.mean():.3f} ms")
            print(f"  Std dt:                    {dts.std():.3f} ms")
            print(f"  Min dt:                    {dts.min():.3f} ms")
            print(f"  Max dt:                    {dts.max():.3f} ms")
            print(f"  Effective loop rate:       {1000.0 / dts.mean():.1f} Hz")
            print(
                f"  Iterations > 6 ms:         {(dts > 6).sum()} "
                f"({(dts > 6).sum() / len(dts) * 100:.2f}%)"
            )
            print(
                f"  Iterations >10 ms:         {(dts > 10).sum()} "
                f"({(dts > 10).sum() / len(dts) * 100:.3f}%)"
            )
        else:
            measured_duration_s = DURATION_S
            print("  Not enough samples for loop timing analysis.")

        if imu is not None:
            report_counts = np.array(imu_report_counts[WARMUP_SAMPLES:])
            value_changes = np.array(imu_value_changes[WARMUP_SAMPLES:])
            read_times = np.array(imu_read_times[WARMUP_SAMPLES:])

            n_polls = len(report_counts)
            n_reports = int(report_counts.sum())
            n_value_changes = int(value_changes.sum())

            if measured_duration_s > 0:
                imu_report_rate = n_reports / measured_duration_s
                value_change_rate = n_value_changes / measured_duration_s
            else:
                imu_report_rate = 0.0
                value_change_rate = 0.0

            timestamp_rate = imu.report_rate_from_timestamps()

            polls_with_0 = int((report_counts == 0).sum())
            polls_with_1 = int((report_counts == 1).sum())
            polls_with_more = int((report_counts > 1).sum())

            print(f"\n{'=' * 60}")
            print("IMU BANDWIDTH - BNO085 GAME_ROTATION_VECTOR")
            print(f"{'=' * 60}")
            print(f"  Polls analysed:            {n_polls}")
            print(f"  Actual IMU reports:        {n_reports}")
            print(f"  Effective report rate:     {imu_report_rate:.1f} Hz")
            print(f"  Timestamp-based rate:      {timestamp_rate:.1f} Hz")
            print()
            print("  Secondary metric:")
            print(f"  Quaternion value changes:  {n_value_changes}")
            print(f"  Value-change rate:         {value_change_rate:.1f} Hz")
            print()
            print("  Reports per poll:")
            print(
                f"    0 reports:               {polls_with_0} "
                f"({polls_with_0 / n_polls * 100:.1f}%)"
            )
            print(
                f"    1 report:                {polls_with_1} "
                f"({polls_with_1 / n_polls * 100:.1f}%)"
            )
            print(
                f"    >1 reports:              {polls_with_more} "
                f"({polls_with_more / n_polls * 100:.1f}%)"
            )

            if len(read_times) > 0:
                print()
                print("  IMU read time per poll:")
                print(f"    Mean:                    {read_times.mean():.3f} ms")
                print(f"    Std:                     {read_times.std():.3f} ms")
                print(f"    Max:                     {read_times.max():.3f} ms")
                print(f"    Median:                  {np.median(read_times):.3f} ms")

            print(f"\n{'=' * 60}")
            print("VERDICT")
            print(f"{'=' * 60}")

            if not imu._hook_installed:
                print("  The internal report counter hook was not installed.")
                print("  -> Cannot measure actual BNO085 reports with this method.")
                print("  -> Only the quaternion value-change metric is available.")
            else:
                if imu_report_rate >= target_hz * 0.95:
                    print(
                        f"  IMU report rate ({imu_report_rate:.0f} Hz) "
                        f">= 95% of target ({target_hz:.0f} Hz)"
                    )
                    print("  -> BNO085 report rate is compatible with 200 Hz control.")
                elif imu_report_rate >= target_hz * 0.85:
                    print(
                        f"  IMU report rate ({imu_report_rate:.0f} Hz) "
                        f"is close to target ({target_hz:.0f} Hz)"
                    )
                    print("  -> 200 Hz is usable, but with some missed/late IMU updates.")
                    print("  -> For robust control, keep encoder at 200 Hz and treat")
                    print("     IMU as an asynchronous measurement.")
                elif imu_report_rate >= target_hz * 0.5:
                    print(
                        f"  IMU report rate ({imu_report_rate:.0f} Hz) "
                        f"is only ~{imu_report_rate / target_hz * 100:.0f}% "
                        f"of target ({target_hz:.0f} Hz)"
                    )
                    print("  -> IMU is the bandwidth-limiting component.")
                    print("  -> Use encoder at 200 Hz and IMU at its native rate,")
                    print("     or lower the full control loop rate.")
                else:
                    print(
                        f"  IMU report rate ({imu_report_rate:.0f} Hz) "
                        f"is far below target ({target_hz:.0f} Hz)"
                    )
                    print("  -> Reconsider IMU interface, report type, or loop architecture.")

            print()
            print("Interpretation:")
            print("  - 'Actual IMU reports' is the metric to use for refresh rate.")
            print("  - 'Quaternion value changes' depends on motion and can be lower")
            print("    when the arm is nearly static.")

        print("\nCSV saved. You can plot the time series later if needed.")

    finally:
        if logger is not None:
            logger.close()

        if hw is not None:
            hw.close()


if __name__ == "__main__":
    main()