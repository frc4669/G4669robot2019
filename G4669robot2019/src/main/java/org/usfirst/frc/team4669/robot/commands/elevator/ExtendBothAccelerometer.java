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

public class ExtendBothAccelerometer extends Command {
  double position;

  public ExtendBothAccelerometer(double positionInches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = -positionInches * Constants.inchToEncoderElevator;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.elevator.getAccelY() < -0.1) {
        Robot.elevator.setVelocity(Robot.elevator.getLeftMotor(),-Robot.elevatorVel);
        Robot.elevator.zeroVelocity(Robot.elevator.getRightMotor());
    } else if (Robot.elevator.getAccelY() > 0.1) {
      Robot.elevator.setVelocity(Robot.elevator.getRightMotor(),-Robot.elevatorVel);
      Robot.elevator.zeroVelocity(Robot.elevator.getLeftMotor());
    } else {
      Robot.elevator.setVelocity(Robot.elevator.getLeftMotor(),-Robot.elevatorVel);
      Robot.elevator.setVelocity(Robot.elevator.getRightMotor(),-Robot.elevatorVel);
      
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double leftPos =  Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor());
    double rightPos = Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor());
    double avgPos = (leftPos + rightPos)/2; 
    return Math.abs(avgPos) < Constants.elevatorTolerance||!Robot.endgameStarted;
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
