import importlib.util
import pathlib
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "tools" / "airlink" / "configure_airlink.py"


def load_module():
    spec = importlib.util.spec_from_file_location("configure_airlink", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load module from {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ConfigureAirlinkTests(unittest.TestCase):
    def test_render_serial_config_uses_udp_server_for_qgc(self):
        module = load_module()

        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = pathlib.Path(tmpdir) / "main.conf"
            config_path = REPO_ROOT / "tools" / "airlink" / "configs" / "serial.yaml"
            rendered_path = module.generate_config(config_path, output_path)
            content = rendered_path.read_text(encoding="utf-8")

        self.assertEqual(rendered_path, output_path)
        self.assertIn("[UartEndpoint px4_serial]", content)
        self.assertIn("Device=/dev/ttyUSB0", content)
        self.assertIn("[UdpEndpoint gcs_listen]", content)
        self.assertIn("Mode=Server", content)
        self.assertIn("Address=0.0.0.0", content)
        self.assertIn("Port=14550", content)
        self.assertNotIn("broadcast", content.lower())

    def test_render_udp_config_keeps_px4_udp_bind_port(self):
        module = load_module()

        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = pathlib.Path(tmpdir) / "main.conf"
            config_path = REPO_ROOT / "tools" / "airlink" / "configs" / "udp.yaml"
            rendered_path = module.generate_config(config_path, output_path)
            content = rendered_path.read_text(encoding="utf-8")

        self.assertEqual(rendered_path, output_path)
        self.assertIn("[UdpEndpoint px4_udp_in]", content)
        self.assertIn("Address=0.0.0.0", content)
        self.assertIn("Port=14540", content)
        self.assertIn("[UdpEndpoint gcs_listen]", content)
        self.assertIn("Mode=Server", content)
        self.assertIn("Port=14550", content)

    def test_serial_mode_requires_serial_device(self):
        module = load_module()

        with self.assertRaisesRegex(ValueError, "serial.device"):
            module.validate_config(
                {
                    "link_mode": "serial",
                    "serial": {"device": "", "baudrate": 921600},
                    "gcs_udp": {"bind_address": "0.0.0.0", "bind_port": 14550},
                }
            )


if __name__ == "__main__":
    unittest.main()
