/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.auto;

import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.commands.elevator.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IncrementalClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IncrementalClimb() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    // establish an if command for conditions of : [1] not being correct distance
    // from climbing [2] facing wrong direction
      int avgPos = (Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor())+Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor()))/2;
      double currentIn = Math.abs(avgPos * Constants.encoderToInchElevator);
      if(currentIn < Constants.incrementalHeightInches){
        addSequential(new ExtendBothElevator(Constants.wheelElevatorOffSet));
        avgPos = (Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor())+Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor()))/2;
        currentIn = Math.abs(avgPos * Constants.encoderToInchElevator);
      }
      if(currentIn < Constants.limitMaxHeight){
        addSequential(new ExtendBothElevator(currentIn + Constants.incrementalHeightInches));
    }
  }
}
