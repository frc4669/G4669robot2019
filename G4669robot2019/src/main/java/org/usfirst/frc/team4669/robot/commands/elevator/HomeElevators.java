/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class HomeElevators extends Command {
  public HomeElevators() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.getLeftMotor().configForwardSoftLimitEnable(false);
    Robot.elevator.getRightMotor().configForwardSoftLimitEnable(false);

    Robot.elevator.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!Robot.elevator.getForwardLimit(Robot.elevator.getLeftMotor())){
      Robot.elevator.percentOutputLeft(0.15);
    } else{
      Robot.elevator.percentOutputLeft(0);
    }
    if(!Robot.elevator.getForwardLimit(Robot.elevator.getRightMotor())){
      Robot.elevator.percentOutputRight(0.15);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.elevator.getForwardLimit(Robot.elevator.getLeftMotor())&&Robot.elevator.getForwardLimit(Robot.elevator.getRightMotor());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.getLeftMotor().configForwardSoftLimitEnable(true);
    Robot.elevator.getRightMotor().configForwardSoftLimitEnable(true);
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
