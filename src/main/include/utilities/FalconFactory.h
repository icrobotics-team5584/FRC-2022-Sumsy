// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/phoenix.h>
#include <memory>

class FalconFactory {
 public:
  static std::unique_ptr<TalonFX> MakePIDFalcon(int canID, double P, double I, double D); 
 
 private:
  FalconFactory();


};
