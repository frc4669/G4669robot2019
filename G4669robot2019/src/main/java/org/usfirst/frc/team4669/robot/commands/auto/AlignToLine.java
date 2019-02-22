/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.auto;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class AlignToLine extends Command {
  public enum Direction {
    FRONT, BACK;
  }
  Direction direction;
  public AlignToLine(Direction direction) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.direction = direction;
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.stop();
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getGyroController());
    switch(direction){
      case FRONT:
        Robot.driveTrain.updateStrafeInput(Robot.frontLineEntries.getX0());
        Robot.driveTrain.enablePIDController(Robot.driveTrain.getStrafeController());
        Robot.driveTrain.setTarget(Robot.driveTrain.getStrafeController(), Constants.pixy2LineHeight/2);
        Robot.driveTrain.setTarget(Robot.driveTrain.getGyroController(), Robot.driveTrain.getAngle());
        break;
      case BACK:
        Robot.driveTrain.updateStrafeInput(Robot.backLineEntries.getX0());
        Robot.driveTrain.enablePIDController(Robot.driveTrain.getStrafeController());
        Robot.driveTrain.setTarget(Robot.driveTrain.getStrafeController(), Robot.driveTrain.getAngle());
        break;
    }
    
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.robotOrientedDrive(Robot.driveTrain.getStrafeOutput(), 0,Robot.driveTrain.getTurnOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveTrain.getPIDDone(Robot.driveTrain.getGyroController()) && Robot.driveTrain.getPIDDone(Robot.driveTrain.getStrafeController());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.disablePIDController(Robot.driveTrain.getStrafeController());
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
