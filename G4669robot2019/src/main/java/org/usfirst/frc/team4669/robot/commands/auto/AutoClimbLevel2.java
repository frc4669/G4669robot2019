/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.auto;

import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.commands.elevator.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoClimbLevel2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoClimbLevel2() {
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

    // Add if statement to not run if not facing right direction and if distance
    // isn't correct
    addSequential(new ExtendBothElevator(Constants.level2HeightInches + Constants.wheelElevatorOffSet));
    addSequential(new DriveElevatorMotionMagic(Constants.climbDriveDistance));
    addSequential(new ExtendRightElevator(Constants.wheelElevatorOffSet));
    addSequential(new DriveElevatorMotionMagic(Constants.climbDriveDistance2));
    addSequential(new ExtendLeftElevator(Constants.wheelElevatorOffSet));

  }
}