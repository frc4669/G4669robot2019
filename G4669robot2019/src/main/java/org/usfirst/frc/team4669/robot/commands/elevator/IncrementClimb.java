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

public class IncrementClimb extends Command {
  double position;
  public IncrementClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    int avgPos = (Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor())+Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor()))/2;
    double currentIn = Math.abs(avgPos * Constants.encoderToInchElevator);
    if(currentIn < Constants.wheelElevatorOffSet + Constants.level2HeightInches - 1){
      extendElevators(Constants.wheelElevatorOffSet + Constants.level2HeightInches);
    } else{
      extendElevators(currentIn + Constants.level2HeightInches);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double leftError = position - Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor());
    double rightError = position - Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor());
    System.out.println("Left Error: "+leftError+" Right Error: "+rightError);
    return (Math.abs(leftError) < Constants.elevatorTolerance&& Math.abs(rightError) < Constants.elevatorTolerance);
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
  }

  private void extendElevators(double inches){
    position = -inches * Constants.inchToEncoderElevator;
    Robot.elevator.stop();
    Robot.elevator.setMagicVelAccel(Robot.elevator.getLeftMotor(), Constants.elevatorVel, Constants.elevatorAccel);
    Robot.elevator.setMagicVelAccel(Robot.elevator.getRightMotor(), Constants.elevatorVel, Constants.elevatorAccel);
    Robot.elevator.setMotionMagicBoth(position);
  }
}
