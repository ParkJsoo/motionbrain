import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "firmware" / "stm32" / "MotionBrainSensor" / "Core" / "Src" / "main.c"


class Stm32MpuRecoveryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAIN.read_text()

    def test_boot_probe_is_bounded_and_reinitializes_i2c_between_attempts(self):
        for fragment in {
            "MPU_BOOT_PROBE_ATTEMPTS 4U",
            "MPU_PROBE_RETRY_DELAY_MS 250U",
            "HAL_I2C_DeInit(&hi2c2)",
            "RecoverI2c2Bus();",
            "MX_I2C2_Init();",
            "InitializeMpu6050WithRetries(&hi2c2, MPU_BOOT_PROBE_ATTEMPTS, 0U)",
        }:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

        self.assertIn("recover_before_first || attempt > 0U", self.source)
        self.assertIn("for (uint32_t attempt = 0; attempt < attempts; ++attempt)", self.source)

    def test_failed_boot_probe_is_retried_without_relaxing_safety_state(self):
        for fragment in {
            "MPU_RUNTIME_RETRY_INTERVAL_MS 5000U",
            "if (!g_mpu_ready &&",
            "InitializeMpu6050WithRetries(&hi2c2, 1U, 1U)",
            "g_mpu_ready = 0;",
            "g_imu_ok = 0;",
            "g_mpu_status = MPU_STATUS_PROBE_FAIL;",
        }:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

        self.assertNotIn("g_imu_ok = 1; // bypass", self.source)

    def test_failed_probe_does_not_report_last_attempted_address_as_active(self):
        probe_start = self.source.index("static uint8_t ProbeMpu6050")
        probe_end = self.source.index("static void PrintFixed3", probe_start)
        probe = self.source[probe_start:probe_end]

        self.assertIn("g_mpu_addr = 0;", probe)
        self.assertEqual(probe.count("g_mpu_addr = addr;"), 1)
        self.assertIn("g_mpu_ready = 1;", probe)


if __name__ == "__main__":
    unittest.main()
