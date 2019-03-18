/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class StartWrist extends Command {
  public StartWrist() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.stop();
    Robot.arm.setMotorPosMagic(Robot.arm.getWristMotor(), Constants.startWrist);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double wristPos = Constants.startWrist;
    double wristError = Math.abs(wristPos - Robot.arm.getEncoderPosition(Robot.arm.getWristMotor()));

    return (Robot.oi.getExtremeRawButton(1) || (wristError < 100));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
