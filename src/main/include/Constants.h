// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace canid {
  constexpr int tfxDriveBaseFrontRight = 1; //check what prefix it should have with liam
  constexpr int tfxDriveBaseBackRight = 2;
  constexpr int tfxDriveBaseFrontLeft = 3;
  constexpr int tfxDriveBaseBackLeft = 4;
  constexpr int spmStorage = 99;
  constexpr int spmIntake = 99;
  constexpr int spmShooter1 = 99;
  constexpr int spmShooter2 = 99;
  constexpr int tfxIntake = 14;
  constexpr int spmPayload = 99;
}

namespace pcm {
  constexpr int solIntakeOut = 0;
  constexpr int solIntakeIn = 1;
}

namespace dio {
  constexpr int lineBreakPayload = 0;
}

