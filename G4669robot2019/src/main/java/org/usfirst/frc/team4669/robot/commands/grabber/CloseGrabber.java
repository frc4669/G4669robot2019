/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code
*/
/* must be accompanied by the FIRST BSD license file in the root directory of
*/
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.grabber;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Opens the robot grabbers.
 */
public class CloseGrabber extends InstantCommand {

    public CloseGrabber() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.grabber);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.grabber.close();
    }

}
