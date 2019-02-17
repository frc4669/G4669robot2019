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

public class ArmAngleSet extends Command {
  double shoulderAngle, elbowAngle, wristAngle;

  public ArmAngleSet(double shoulderAngle, double elbowAngle, double wristAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
    this.shoulderAngle = shoulderAngle;
    this.elbowAngle = elbowAngle;
    this.wristAngle = wristAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting Arm Motion Magic");
    Robot.arm.stop();
    System.out.println("Setting Shoulder Target");
    Robot.arm.setToAngle(Robot.arm.getShoulderMotor(), shoulderAngle, false);
    System.out.println("Setting Elbow Target");
    Robot.arm.setToAngle(Robot.arm.getElbowMotor(), elbowAngle, false);
    System.out.println("Setting Wrist Target");
    Robot.arm.setToAngle(Robot.arm.getWristMotor(), wristAngle, false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double shoulderPos = shoulderAngle * Constants.encoderTicksPerRotation * Constants.shoulderGearRatio / 360;
    double elbowPos = elbowAngle * Constants.encoderTicksPerRotation * Constants.elbowGearRatio / 360;
    double wristPos = wristAngle * Constants.encoderTicksPerRotation * Constants.wristGearRatio / 360;

    double shoulderError = Math.abs(shoulderPos - Robot.arm.getEncoderPosition(Robot.arm.getShoulderMotor()));
    double elbowError = Math.abs(elbowPos - Robot.arm.getEncoderPosition(Robot.arm.getElbowMotor()));
    double wristError = Math.abs(wristPos - Robot.arm.getEncoderPosition(Robot.arm.getWristMotor()));

    return (Robot.oi.getExtremeRawButton(1) || (shoulderError < 10 && elbowError < 10 && wristError < 10));

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
