/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class MecanumDriveCommand extends Command {

  boolean turnRunning = false;

  public MecanumDriveCommand() {
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
    // Turns the Robot to D-Pad angle
    // if (Robot.f310.getDPadPOV() != -1 && !turnRunning) {
    // double turnAngle = Robot.f310.getDPadPOV();
    // Robot.driveTrain.enableTurnPID();
    // Robot.driveTrain.setTurnAngle(turnAngle);
    // turnRunning = true;
    // } else if (turnRunning && Robot.driveTrain.getTurnDone()) {
    // Robot.driveTrain.disableTurnPID();
    // Robot.driveTrain.stop();
    // turnRunning = false;
    // } else if (turnRunning) {
    // Robot.driveTrain.driveFrontLeft(Robot.driveTrain.getTurnOutput());
    // Robot.driveTrain.driveRearLeft(Robot.driveTrain.getTurnOutput());
    // Robot.driveTrain.driveFrontRight(-Robot.driveTrain.getTurnOutput());
    // Robot.driveTrain.driveRearRight(-Robot.driveTrain.getTurnOutput());
    // }

    // else {
    // Robot.driveTrain.fieldOrientedDrive(Robot.f310.getLeftX(),
    // Robot.f310.getLeftY(), Robot.f310.getRightX(),
    // Robot.driveTrain.getGyroAngle());
    double strafe = Robot.f310.getLeftX();
    double forward = Robot.f310.getLeftY();
    double rotation = Robot.f310.getRightX();
    // Robot.driveTrain.robotDrive(forward, strafe, rotation);
    Robot.driveTrain.robotOrientedDrive(strafe, forward, rotation);
    if (strafe != 0)
      System.out.println("Forward Speed: " + forward);
    if (forward != 0)
      System.out.println("Strafe Speed: " + strafe);
    if (rotation != 0)
      System.out.println("Rotation: " + rotation);
    // }
    // Robot.driveTrain.feedWatchdog();
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
