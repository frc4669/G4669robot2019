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

public class DriveElevatorMotionMagic extends Command {
  public enum Sensor {
    RIGHT, LEFT, DISABLED;
  }

  Sensor sensorStatus;

  
  public DriveElevatorMotionMagic(double timeout, Sensor sensorStatus){
      // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setTimeout(timeout);
    this.sensorStatus = sensorStatus;
    requires(Robot.elevator);
  }

  public DriveElevatorMotionMagic(double timeout) {
    this(timeout, Sensor.DISABLED);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.stop();
    // Robot.elevator.zeroWheelEncoder();
    // Robot.elevator.setMotionMagic(Robot.elevator.getWheelMotor(), position);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.percentOutputWheel(0.35);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    switch(sensorStatus){
      case RIGHT:
        if(Robot.elevator.getRightSensorUpdate()<Constants.climberRightSensorTol){
          return true;
        }
        break;
      case LEFT:
        if(Robot.elevator.getLeftSensorUpdate()<Constants.climberLeftSensorTol){
          return true;
        }
          break;
      case DISABLED:
        break;
    }
    return isTimedOut();
    // return (Math
    //     .abs(position - Robot.elevator.getEncoderPos(Robot.elevator.getWheelMotor())) < Constants.driveTolerance);
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
