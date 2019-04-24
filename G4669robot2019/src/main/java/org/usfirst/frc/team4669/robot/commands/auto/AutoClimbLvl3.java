/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.auto;

import org.usfirst.frc.team4669.robot.commands.driveTrain.StrafeStraight;
import org.usfirst.frc.team4669.robot.commands.elevator.DriveElevatorMotionMagic;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendBothElevator;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendLeftElevator;
import org.usfirst.frc.team4669.robot.commands.elevator.ExtendRightElevator;
import org.usfirst.frc.team4669.robot.commands.elevator.IncrementClimb;
import org.usfirst.frc.team4669.robot.commands.elevator.DriveElevatorMotionMagic.Sensor;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoClimbLvl3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoClimbLvl3() {
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
    addSequential(new IncrementClimb());
    addSequential(new IncrementClimb());
    addSequential(new IncrementClimb());
    addSequential(new DriveElevatorMotionMagic(2, Sensor.RIGHT));
    addSequential(new ExtendRightElevator(0));
    addSequential(new StrafeStraight(1.5,true));
    addSequential(new ExtendRightElevator(Constants.wheelElevatorOffSet));
    addSequential(new DriveElevatorMotionMagic(5, Sensor.LEFT));
    addSequential(new ExtendBothElevator(0));

  }
}
