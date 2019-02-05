/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoClimbLevel3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoClimbLevel3() {
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
    addSequential(new ExtendBothElevator(Constants.level3HeightInches + Constants.wheelElevatorOffSet));
    addSequential(new DriveElevatorMotionMagic(Constants.climbDriveDistance));
    addSequential(new ExtendRightElevator(Constants.wheelElevatorOffSet));
    addSequential(new DriveElevatorMotionMagic(Constants.climbDriveDistance2));
    addSequential(new ExtendLeftElevator(Constants.wheelElevatorOffSet));
    addSequential(new StrafeMotionMagic(Constants.climbDriveDistance3));
  }
}
