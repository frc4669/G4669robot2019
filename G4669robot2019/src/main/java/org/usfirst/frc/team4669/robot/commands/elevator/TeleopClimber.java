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
    int leftPos = 0, rightPos = 0;

    public TeleopClimber() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.elevator.stop();
        leftPos = Robot.elevator.getEncoderPos(Robot.elevator.getLeftMotor());
        rightPos = Robot.elevator.getEncoderPos(Robot.elevator.getRightMotor());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.elevator.setMotionMagic(Robot.elevator.getLeftMotor(), leftPos);
        Robot.elevator.setMotionMagic(Robot.elevator.getRightMotor(), rightPos);
        if (Robot.f310.getRightTrigger() > 0.2)
            wheelPower = 0.8 * Robot.f310.getRightTrigger();
        else if (Robot.f310.getLeftTrigger() > 0.2)
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
