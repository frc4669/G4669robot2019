/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.misc;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceWrapper implements PIDSource {

  private double input;
  private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;

  public PIDSourceWrapper(double input) {
    this.input = input;
  }

  public void setInput(double input){
    this.input = input;
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    pidSourceType = pidSource;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }
  

  @Override
  public double pidGet() {
    return input;
  }

}
