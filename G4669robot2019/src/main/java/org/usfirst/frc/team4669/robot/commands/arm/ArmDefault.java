/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArmDefault extends Command {
  public ArmDefault() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.stop();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Keeps the arm joints at 0 velocity so it doesn't move
    double wristPower = Robot.oi.extremeZ();
    double elbowPower = Robot.oi.extremeX();
    double shoulderPower = Robot.oi.extremeY();

    if (Math.abs(wristPower) > Math.abs(elbowPower) && Math.abs(wristPower) > Math.abs(shoulderPower)) {
      elbowPower = 0;
      shoulderPower = 0;
    }
    if (Math.abs(elbowPower) > Math.abs(shoulderPower) && Math.abs(elbowPower) > Math.abs(wristPower)) {
      wristPower = 0;
      shoulderPower = 0;
    }
    if (Math.abs(shoulderPower) > Math.abs(elbowPower) && Math.abs(shoulderPower) > Math.abs(wristPower)) {
      elbowPower = 0;
      wristPower = 0;
    }
    if (wristPower == 0 && elbowPower == 0 && shoulderPower == 0) {
      Robot.arm.zeroVelocity(Robot.arm.getShoulderMotor());
      Robot.arm.zeroVelocity(Robot.arm.getElbowMotor());
      Robot.arm.zeroVelocity(Robot.arm.getWristMotor());
    } else {
      Robot.arm.motorControl(shoulderPower, elbowPower, wristPower);
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
    Robot.arm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
