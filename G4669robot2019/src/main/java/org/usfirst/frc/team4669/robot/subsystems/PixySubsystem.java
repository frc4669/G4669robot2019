/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import java.util.Date;
import org.usfirst.frc.team4669.robot.pixy.Pixy;
import org.usfirst.frc.team4669.robot.pixy.PixyBlock;
import org.usfirst.frc.team4669.robot.pixy.PixyTune;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class PixySubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Pixy pixy;
  private PixyTune pixyTuner;

  public PixySubsystem(){
    if(Pixy.ensureAvailable(0x10AE5988)){
			pixy = new Pixy(0x10AE5988);
			pixyTuner = new PixyTune(pixy);
		}

  }

  public PixyBlock getBlock(){
    if(pixy.getBlocks().size()>0)
      return pixy.getBlocks().get(0);
    else return null;
  }

  public boolean getPixyAvailable(){
    return Pixy.ensureAvailable(0x10AE5988);
  }

  public int getX(){
    return getBlock().x;
  }

  public int getY(){
    return getBlock().y;
  }

  public int getArea(){
    return getBlock().width*getBlock().height;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
