/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.OpenGrabber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid leftDoubleAction = new DoubleSolenoid(RobotMap.leftSolenoidForward,
      RobotMap.leftSolenoidReverse);
  private DoubleSolenoid rightDoubleAction = new DoubleSolenoid(RobotMap.rightSolenoidForward,
      RobotMap.rightSolenoidReverse);
  // Channels are reversed because left and right sides are mirrored

  public void open() {
    leftDoubleAction.set(DoubleSolenoid.Value.kForward);
    rightDoubleAction.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    leftDoubleAction.set(DoubleSolenoid.Value.kReverse);
    rightDoubleAction.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // Not using default commands
  }

}
