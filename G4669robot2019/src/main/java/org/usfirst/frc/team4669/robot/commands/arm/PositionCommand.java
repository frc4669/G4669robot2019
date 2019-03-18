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

public class PositionCommand extends Command {
  double shoulderAngle, elbowAngle, wristAngle;
  public static ArmData lastCommand,lastPosition;

  protected ArmData ball, hook;
  boolean running = false;


  public PositionCommand(ArmData ball, ArmData hook) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
    
    this.ball = ball;
    this.hook = hook;
  }

  public PositionCommand(ArmData ball, ArmData hook, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
    setTimeout(timeout);
    this.ball = ball;
    this.hook = hook;
  }

  public PositionCommand(ArmData armData) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this(armData,armData);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting Arm Motion Magic");
    Robot.arm.stop();
    ArmData curData = null;
    double shoulderAngle, elbowAngle, wristAngle;
    boolean ballMode = false;
    if(Robot.toggleBallMode){
      curData = ball;
      ballMode = true;
    } else{
      curData = hook;
      ballMode = false;
    }
    double[] armAngles = Robot.arm.calculateAngles(curData.x, curData.xCorrect, curData.y, curData.yCorrect, curData.targetGrabberAngle, curData.angleCorrect,curData.flipUp, ballMode);
    if (armAngles != null) {
      running = true;
      shoulderAngle = armAngles[0];
      elbowAngle = armAngles[1];
      wristAngle = armAngles[2];
      if(ArmData.ballList.indexOf(curData)!=-1||ArmData.hatchList.indexOf(curData)!=-1){
        lastCommand = curData;
      }
      lastPosition = curData;
      System.out.println("Setting Elbow Target");
      Robot.arm.setToAngle(Robot.arm.getElbowMotor(), elbowAngle);
      System.out.println("Setting Shoulder Target");
      Robot.arm.setToAngle(Robot.arm.getShoulderMotor(), shoulderAngle);
      System.out.println("Setting Wrist Target");
      Robot.arm.setToAngle(Robot.arm.getWristMotor(), wristAngle);
    }
    else{
      running = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!running)
      return true;
    double shoulderPos = shoulderAngle * Constants.encoderTicksPerRotation * Constants.shoulderGearRatio / 360;
    double elbowPos = elbowAngle * Constants.encoderTicksPerRotation * Constants.elbowGearRatio / 360;
    double wristPos = wristAngle * Constants.encoderTicksPerRotation * Constants.wristGearRatio / 360;

    double shoulderError = Math.abs(shoulderPos - Robot.arm.getEncoderPosition(Robot.arm.getShoulderMotor()));
    double elbowError = Math.abs(elbowPos - Robot.arm.getEncoderPosition(Robot.arm.getElbowMotor()));
    double wristError = Math.abs(wristPos - Robot.arm.getEncoderPosition(Robot.arm.getWristMotor()));

    System.out.println("Shoulder error: " + shoulderError +  " Elbow error: " + elbowError + "Wrist error: " + wristError);
    return (isTimedOut()||Robot.oi.getExtremeRawButton(1) || (shoulderError < 300 && elbowError < 300 && wristError < 300));

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.stop();
    System.out.println("Position Command ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
