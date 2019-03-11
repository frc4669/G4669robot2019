/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
// import org.usfirst.frc.team4669.robot.commands.OpenGrabber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid doubleAction = new DoubleSolenoid(RobotMap.solenoidForward, RobotMap.solenoidReverse);
  private Compressor compressor = new Compressor();
  public void open() {
    doubleAction.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    doubleAction.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean getGrabberOpen() {
    if (doubleAction.get() == DoubleSolenoid.Value.kForward) {
      return true;
    }
    return false;
  }

  public void startCompressor(){
    compressor.start();
  }

  public void stopCompressor(){
    compressor.stop();
  }

  public boolean isCompressorRunning(){
    return compressor.enabled();
  }

  public boolean isPressureLow(){
    return compressor.getPressureSwitchValue();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // Not using default commands
  }

}
