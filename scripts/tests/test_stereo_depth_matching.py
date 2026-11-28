#!/usr/bin/env python3
"""The stereo depth's two searches (specification K94) as the node joins them, and the accuracy tool's copy."""
import re
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]


def joined(text: str) -> str:
    """The statement that joins the near and the far answer, without its whitespace."""
    start = text.index("if (near <= 0 || near > 16 * half_column) {")
    return re.sub(r"\s+", "", text[start:text.index("}", text.index("} else if", start) + 1) + 1])


class StereoDepthMatchingTest(unittest.TestCase):
    def test_each_search_answers_for_its_own_depths(self) -> None:
        # The far search's answer stood wherever the near one was silent, and the near one's wherever the far found
        # nothing: a wall 0.6 m beside the vehicle was laid 1.3 m off, the floor under the resting vehicle 1.5 m
        # below itself (r4319: 14 displaced sheets, 4 by this rule on the same frames and poses).
        node = (REPOSITORY / "drone_city_nav/src/stereo_depth_node_match.cpp").read_text(encoding="utf-8")
        rule = joined(node)
        self.assertEqual(
            rule,
            "if(near<=0||near>16*half_column){far=std::int16_t{-16};}"
            "elseif(near>near_threshold_16){far=static_cast<std::int16_t>(std::min(32767,2*near));}",
        )
        # The near search over half frames padded on the left by its search: unpadded it is silent in its first
        # 128 columns, a fifth of the frame.
        self.assertEqual(node.count("near_disparities_, 0,\n                     cv::BORDER_REPLICATE);"), 2)
        self.assertIn("near_disparity.at<std::int16_t>(row / 2, half_column + near_disparities_)", node)

    def test_the_accuracy_tool_matches_as_the_node(self) -> None:
        node = (REPOSITORY / "drone_city_nav/src/stereo_depth_node_match.cpp").read_text(encoding="utf-8")
        tool = (REPOSITORY / "drone_city_nav/tools/stereo_depth_accuracy.cpp").read_text(encoding="utf-8")
        self.assertEqual(joined(tool), joined(node))
        self.assertEqual(tool.count("kNearDisparities, 0,\n                       cv::BORDER_REPLICATE);"), 2)
        self.assertIn("near_disparity.at<std::int16_t>(row / 2, half_column + kNearDisparities)", tool)


if __name__ == "__main__":
    unittest.main()
