import unittest


class AerialManipulationPackageTests(unittest.TestCase):
    def test_package_imports(self):
        import aerial_manipulation_benchmark

        self.assertIsNotNone(aerial_manipulation_benchmark)


if __name__ == "__main__":
    unittest.main()
