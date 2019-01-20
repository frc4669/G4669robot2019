/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class LineAlignment extends Command {

  public boolean leftS = true;
  public boolean rightS = true;
  public boolean midS = true;

  public LineAlignment() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // check sensor code and adjust bool//
    if (leftS) {
      Robot.driveTrain.polarDrive(.3, 45, .5);
    } else if (rightS) {
      Robot.driveTrain.polarDrive(.3, -45, -.5);
    } else if (midS) {
      Robot.driveTrain.polarDrive(.3, 0, 0);
    } else {
      // changed by Tony 1/12/2019
      Robot.driveTrain.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override

  // this should be overiden when the robot is controlled - Tony 1/12/2019
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
