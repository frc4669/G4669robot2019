/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.commands.grabber.CloseGrabber;
import org.usfirst.frc.team4669.robot.commands.grabber.OpenGrabber;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ToggleBallHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ToggleBallHatch() {
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
    if(PositionCommand.lastCommand==ArmData.hookStart&&!Robot.toggleBallMode){
      addSequential(new OpenGrabber());
      addSequential(new PositionCommand(ArmData.hookToBall));
      Robot.toggleBallMode = true;
      addSequential(new PathCommand(0));
      addSequential(new CloseGrabber());
    } else if(PositionCommand.lastCommand==ArmData.ballPickup&&Robot.toggleBallMode){
      addSequential(new OpenGrabber());
      Robot.toggleBallMode = false;
      addSequential(new PositionCommand(ArmData.ballToHatch));
      addSequential(new PathCommand(0));

    }

  }
}
