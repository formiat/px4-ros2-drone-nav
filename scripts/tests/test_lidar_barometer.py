"""The lidar-inertial estimator's height (specification K45): the lidar profile bridges the vehicle's barometer, the
estimator node reads it, the record carries it and the replay takes it."""

import pathlib
import unittest

REPOSITORY = pathlib.Path(__file__).resolve().parents[2]


class LidarBarometerContract(unittest.TestCase):
    def test_the_lidar_profile_bridges_the_barometer_when_no_image_source_relays_it(self):
        profile = (REPOSITORY / "drone_city_nav/launch/sensor_profile.py").read_text()
        self.assertIn("air_pressure_sensor/air_pressure", profile)
        self.assertIn("@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure", profile)
        self.assertIn(":=/barometer/air_pressure", profile)
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text()
        self.assertIn("if not cameras_mounted:", launch)
        self.assertIn("lidar_barometer_bridge(gazebo_world_name, gazebo_model_name)", launch)

    def test_the_node_reads_the_barometer_and_declares_its_share(self):
        node = (REPOSITORY / "drone_city_nav/src/lidar_inertial_odometry_node.cpp").read_text()
        self.assertIn('declare_parameter<std::string>("air_pressure_topic", "/barometer/air_pressure")', node)
        self.assertIn('declare_parameter<double>("height_prior_share"', node)
        self.assertIn("kPressurePerMetrePa{12.013}", node)

    def test_the_record_and_the_replay_carry_the_barometer(self):
        self.assertIn("/barometer/air_pressure", (REPOSITORY / "tools/lio/record_flight.sh").read_text())
        self.assertIn('"/barometer/air_pressure"', (REPOSITORY / "tools/lio/extract_bag.py").read_text())
        replay = (REPOSITORY / "tools/lio/replay_lio.cpp").read_text()
        self.assertIn('k == "baro"', replay)
        self.assertIn('k == "heightshare"', replay)


if __name__ == "__main__":
    unittest.main()
