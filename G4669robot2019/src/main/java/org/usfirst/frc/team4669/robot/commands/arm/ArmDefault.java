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

public class ArmDefault extends Command {
  boolean holdShoulder = false, holdElbow = false, holdWrist = false;
  double shoulderPos = 0, elbowPos = 0, wristPos = 0;
  double wristPower = 0, elbowPower = 0, shoulderPower = 0;
  public ArmDefault() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.stop();
    Robot.arm.setMotorPosMagic(Robot.arm.getShoulderMotor(), Constants.startShoulder);
    Robot.arm.setMotorPosMagic(Robot.arm.getElbowMotor(), Constants.startElbow);
    Robot.arm.setMotorPosMagic(Robot.arm.getWristMotor(), Constants.startWrist);
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.oi.getExtremeRawButton(1)){
      wristPower = 0.3*Robot.oi.extremeZ();
      elbowPower = 0.5*Robot.oi.extremeX();
      shoulderPower = 0.4*Robot.oi.extremeY();
    } else{
      wristPower = 0;
      elbowPower = 0;
      shoulderPower = 0;
    }

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
    if(shoulderPower == 0){
      if(!holdShoulder){
        shoulderPos = Robot.arm.getEncoderPosition(Robot.arm.getShoulderMotor());
        holdShoulder = true;
      }
      Robot.arm.setPosition(Robot.arm.getShoulderMotor(),shoulderPos);
    }
    else{
      Robot.arm.motorControl(Robot.arm.getShoulderMotor(), shoulderPower);
      holdShoulder = false;
    } 
    if(elbowPower == 0){
      if(!holdElbow){
        elbowPos = Robot.arm.getEncoderPosition(Robot.arm.getElbowMotor());
        holdElbow = true;
      }
      Robot.arm.setPosition(Robot.arm.getElbowMotor(),elbowPos);
        }
    else{
      Robot.arm.motorControl(Robot.arm.getElbowMotor(), elbowPower);
      holdElbow = false;
    }
    if(wristPower == 0){
      if(!holdWrist){
        wristPos = Robot.arm.getEncoderPosition(Robot.arm.getWristMotor());
        holdWrist = true;
      }
      Robot.arm.setPosition(Robot.arm.getWristMotor(), wristPos);
    }
    else{
      Robot.arm.motorControl(Robot.arm.getWristMotor(), wristPower);
      holdWrist = false;
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
    holdWrist = holdElbow = holdShoulder = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
