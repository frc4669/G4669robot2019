/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Rocket1Front extends CommandGroup {
  /**
   * Add your docs here.
   */

  private ArmToPosition hatch1F = new ArmToPosition(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.hatch1Height, 2, 0, -8, false, false),
                        ball1F = new ArmToPosition(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.ball1Height, 0, 0, 0, false, true);
  public Rocket1Front() {
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
    
    
  }
  @Override
  public void execute(){
    if(Robot.toggleBallMode){
      ball1F.start();
    } else{
      hatch1F.start();
    }
  }
}
