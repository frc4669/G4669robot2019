/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.driveTrain;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class JoystickDrive extends Command {

  boolean turnRunning = false;

  public JoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double strafe = 0.8 * Robot.f310.getLeftX();
    double forward = 0.8 * Robot.f310.getLeftY();
    double rotation = 0.8 * Robot.f310.getRightX();
    if (Robot.f310.getButton(F310.rightShoulderButton)) {
      strafe = 0.5 * Robot.f310.getLeftX();
      forward = 0.3 * Robot.f310.getLeftY();
      rotation = 0.3 * Robot.f310.getRightX();
    }
    if (Robot.f310.getButton(F310.leftShoulderButton)) {
      if(Robot.driveTrain.getFrontDistance()<14){
        forward = 0;
      }
    }

    Robot.driveTrain.robotOrientedDrive(strafe, forward, rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
