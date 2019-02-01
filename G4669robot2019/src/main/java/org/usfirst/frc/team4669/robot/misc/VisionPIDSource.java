/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.misc;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class VisionPIDSource implements PIDSource {

  public enum BallAlign {
    DISTANCE, TURN;
  }

  private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
  private BallAlign alignType;
  private VisionEntries visionEntries = new VisionEntries();

  public VisionPIDSource(BallAlign align) {
    alignType = align;
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    pidSourceType = pidSource;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }

  public BallAlign getBallAlignType() {
    return alignType;
  }

  public void setBallAlignType(BallAlign align) {
    alignType = align;
  }

  public boolean isObjectDetected() {
    return visionEntries.isObjectDetected();
  }

  @Override
  public double pidGet() {
    switch (alignType) {
    case DISTANCE:
      return visionEntries.getDistance();
    case TURN:
      return visionEntries.getX();
    default:
      return 0;
    }
  }
}
