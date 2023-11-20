/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "utils/utils.h"
#include <gtest/gtest.h>

using namespace connector_utils;

TEST(Utils, CheckParamIncludes) {
  EXPECT_TRUE(CheckParamIncludes("test_string_alpha/substring", "substring"));
  EXPECT_FALSE(CheckParamIncludes("test_string_alpha/substring", "string_alpha"));
}
TEST(Utils, CheckRange) {
  double upperRange = 1.0;
  double lowerRange = 0.0;
  EXPECT_TRUE(CheckRange(lowerRange, upperRange, (upperRange + lowerRange) / 2.0));
  EXPECT_FALSE(CheckRange(lowerRange, upperRange, upperRange + 0.2));
  EXPECT_FALSE(CheckRange(lowerRange, upperRange, lowerRange - 0.2));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  return RUN_ALL_TESTS();
}
