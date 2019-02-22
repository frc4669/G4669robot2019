/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.auto;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTableEntry;

public class AlignToBall extends Command {

  int centerX = 160;
  int centerY = 100;

  public AlignToBall() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.stop();
    Robot.driveTrain.updateVisionTurnInput(Robot.visionEntries.getX());
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getVisionTurnController());
    Robot.driveTrain.setTarget(Robot.driveTrain.getVisionTurnController(), centerX);
    Robot.driveTrain.updateVisionDriveInput(Robot.visionEntries.getHeight());
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getVisionDistanceController());
    Robot.driveTrain.setTarget(Robot.driveTrain.getVisionDistanceController(), 150);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driveOutput = Robot.driveTrain.getVisionDistanceOutput();
    double turnOutput = Robot.driveTrain.getVisionTurnOutput();
    Robot.driveTrain.robotOrientedDrive(driveOutput, 0, turnOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!Robot.visionEntries.isObjectDetected()
        || Robot.driveTrain.getPIDDone(Robot.driveTrain.getVisionTurnController())
            && Robot.driveTrain.getPIDDone(Robot.driveTrain.getVisionDistanceController()));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.disablePIDController(Robot.driveTrain.getVisionDistanceController());
    Robot.driveTrain.disablePIDController(Robot.driveTrain.getVisionTurnController());
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
