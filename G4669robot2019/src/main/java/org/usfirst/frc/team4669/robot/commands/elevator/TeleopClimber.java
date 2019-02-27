package org.usfirst.frc.team4669.robot.commands.elevator;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
*
*/
public class TeleopClimber extends Command {
    double wheelPower = 0;

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
        if (Robot.oi.getLeftRawButton(6))
            Robot.elevator.percentOutputLeft(0.6);
        else if (Robot.oi.getLeftRawButton(7))
            Robot.elevator.percentOutputLeft(-0.6);
        else
            Robot.elevator.zeroVelocity(Robot.elevator.getLeftMotor());
        ;
        if (Robot.oi.getLeftRawButton(11))
            Robot.elevator.percentOutputRight(0.6);
        else if (Robot.oi.getLeftRawButton(10))
            Robot.elevator.percentOutputRight(-0.6);
        else
            Robot.elevator.zeroVelocity(Robot.elevator.getRightMotor());
        if (Robot.f310.getRightTrigger() > 0)
            wheelPower = 0.8 * Robot.f310.getRightTrigger();
        else if (Robot.f310.getLeftTrigger() > 0)
            wheelPower = 0.8 * -Robot.f310.getLeftTrigger();
        else
            wheelPower = 0;
        Robot.elevator.percentOutputWheel(wheelPower);

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