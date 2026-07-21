import unittest


class Figure8TrackingPackageTests(unittest.TestCase):
    def test_package_imports(self):
        import figure8_tracking_benchmark

        self.assertIsNotNone(figure8_tracking_benchmark)


if __name__ == "__main__":
    unittest.main()
