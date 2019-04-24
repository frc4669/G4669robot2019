/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.driveTrain;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Add your docs here.
 */
public class StrafeStraight extends TimedCommand {
  /**
   * Add your docs here.
   */
  double angle;
  boolean useSensor;
  public StrafeStraight(double timeout, boolean useSensor) {
    super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    this.useSensor = useSensor;
  }

  public StrafeStraight(double timeout) {
    this(timeout,false);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    angle = Robot.driveTrain.getAngleNormalized();
    Robot.driveTrain.stop();
    Robot.driveTrain.configPIDController(Robot.driveTrain.getGyroController(), 0, 360, true, 0.5, 1);
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getGyroController());
    Robot.driveTrain.setTarget(Robot.driveTrain.getGyroController(), angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.robotOrientedDrive(0.6, 0, Robot.driveTrain.getTurnOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(useSensor){
      return Robot.elevator.getMidSensorUpdate()<Constants.climberMidSensorTol||super.isFinished();
    }
    return super.isFinished();
  }


  // Called once after isFinished returns true
  @Override
  protected void end() {

    Robot.driveTrain.disablePIDController(Robot.driveTrain.getGyroController());
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
