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
        if (Robot.oi.getLeftRawButton(10))
            Robot.elevator.stop();
        else if (Robot.oi.getLeftRawButton(8)) {
            if (Robot.elevator.getAccelY() < -0.1) {
                Robot.elevator.percentOutputLeft(0.2);
                Robot.elevator.zeroVelocity(Robot.elevator.getRightMotor());
            } else if (Robot.elevator.getAccelY() > 0.1) {
                Robot.elevator.percentOutputRight(0.2);
                Robot.elevator.zeroVelocity(Robot.elevator.getLeftMotor());
            } else {
                Robot.elevator.percentOutputRight(0.2);
                Robot.elevator.percentOutputLeft(0.2);
            }
        } else if (Robot.oi.getLeftRawButton(6))
            Robot.elevator.percentOutputLeft(0.2);
        else if (Robot.oi.getLeftRawButton(11))
            Robot.elevator.percentOutputRight(0.2);
        else {
            Robot.elevator.zeroVelocity(Robot.elevator.getLeftMotor());
            Robot.elevator.zeroVelocity(Robot.elevator.getRightMotor());
        }

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
