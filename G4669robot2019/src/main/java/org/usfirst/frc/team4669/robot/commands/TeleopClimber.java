package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
*
*/
public class TeleopClimber extends Command {

    public TeleopClimber() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.elevator.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.elevator.zeroVelocity(Robot.elevator.getLeftMotor());
        Robot.elevator.zeroVelocity(Robot.elevator.getRightMotor());

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.elevator.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

}
