/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.elevator;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class ExtendRightElevator extends Command {
  double position;

  public ExtendRightElevator(double positionInches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = -positionInches * Constants.inchToEncoderElevator;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.stop();
    Robot.elevator.setMagicVelAccel(Robot.elevator.getRightMotor(), Constants.elevatorVel, Constants.elevatorAccel);
    Robot.elevator.setMotionMagic(Robot.elevator.getRightMotor(), position);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math
        .abs(position - Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor())) < Constants.elevatorTolerance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
