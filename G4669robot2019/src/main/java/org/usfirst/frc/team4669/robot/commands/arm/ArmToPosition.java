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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmToPosition extends CommandGroup {

  public ArmToPosition(double x, double y, double targetGrabberAngle, boolean flipUp, boolean ballMode) {
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
    double shoulderAngle, elbowAngle, wristAngle;
    // shoulderAngle = Robot.arm.targetToAngleShoulder(x, y);
    // elbowAngle = Robot.arm.targetToAngleElbow(x, y);
    if (Robot.arm.calculateAngles(x, y, targetGrabberAngle, flipUp, ballMode) != null) {
      double[] armAngles = Robot.arm.calculateAngles(x, y, targetGrabberAngle, flipUp, ballMode);
      shoulderAngle = armAngles[0];
      elbowAngle = armAngles[1];
      wristAngle = armAngles[2];
      if (!(shoulderAngle != shoulderAngle || elbowAngle != elbowAngle))
        addSequential(new ArmAngleSet(shoulderAngle, elbowAngle, wristAngle));
      SmartDashboard.putNumber("Wrist Angle", wristAngle);
    }
  }
}
