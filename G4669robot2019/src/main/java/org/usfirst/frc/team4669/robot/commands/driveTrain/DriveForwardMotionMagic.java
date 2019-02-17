package org.usfirst.frc.team4669.robot.commands.driveTrain;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardMotionMagic extends Command {

    private double distance;
    private double timeOut = 0;

    public DriveForwardMotionMagic(double distance) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.distance = distance;
        requires(Robot.driveTrain);
    }

    public DriveForwardMotionMagic(double distance, double timeOut) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.timeOut = timeOut;
        this.distance = distance;
        requires(Robot.driveTrain);
    }

    // Called once when the command executes
    protected void initialize() {
        Robot.driveTrain.zeroEncoders();
        if (timeOut != 0) {
            setTimeout(timeOut);
        }
        System.out.println(distance);
        Robot.driveTrain.driveMotionMagic(distance * Constants.inchToEncoderDrive); // Converts inches to encoder ticks
        System.out.println("Target Position: " + distance * Constants.inchToEncoderDrive);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return ((Math
                .abs(distance * Constants.inchToEncoderDrive
                        - Robot.driveTrain.getFrontRightEncoder()) < Constants.driveTolerance
                && Math.abs(distance * Constants.inchToEncoderDrive
                        - Robot.driveTrain.getFrontLeftEncoder()) < Constants.driveTolerance)
                || Robot.f310.getButton(F310.redButton));
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
