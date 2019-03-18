/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathCommand(int end) {
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
    int index;
    if(PositionCommand.lastCommand == null){
      System.out.println("Last command is null");
      if(Robot.toggleBallMode){
        index = 1;
      } else{
        index = 0;
      }
    } else{
        index = ArmData.ballList.indexOf(PositionCommand.lastCommand);
        if(index == -1){
          index = ArmData.hatchList.indexOf(PositionCommand.lastCommand);
        }
    }
    System.out.println("Start: " + index + "End: " + end);
      if(index < end){
        for(int x = index+1; x<=end; x++){
          ArmData data;
          if(!Robot.toggleBallMode){
            data = ArmData.hatchList.get(x);
          } else{
            data = ArmData.ballList.get(x);
          }
          addSequential(new PositionCommand(data));
        }
      }
      if(index > end){
        for(int x = index-1; x>=end; x--){
          ArmData data;
          if(!Robot.toggleBallMode){
            data = ArmData.hatchList.get(x);
          } else{
            data = ArmData.ballList.get(x);
          }
          addSequential(new PositionCommand(data));
        }
      }
  }
}
