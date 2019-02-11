package org.usfirst.frc.team4669.robot.commands;

import java.io.File;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
*
*/
public class PathfinderTest extends Command {

    // Trajectory trajectory;
    // Trajectory.Config config;
    // TankModifier modifier;
    EncoderFollower leftFollower;
    EncoderFollower rightFollower;
    Notifier followerNotifier;
    double l, r, turn;
    double gyro_heading, desired_heading, angleDifference;
    static final String pathName = "Drive 5FT";

    public PathfinderTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.driveTrain.zeroEncoders();
        Robot.driveTrain.calibrateGyro();
        Robot.driveTrain.resetGyro();
        System.out.println("Attempting to get trajectories from CSV File");
        /* Getting the left and right trajectories from the CSV files */
        Trajectory rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
        Trajectory leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");

        System.out.print("Got the trajectories successfully");

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);

        // Must configure PIDVA on the following line
        leftFollower.configureEncoder(Robot.driveTrain.getFrontLeftEncoder(), Constants.encoderTicksPerRotation,
                Constants.wheelDiameter);
        leftFollower.configurePIDVA(0.3, 0, 0, 1 / Constants.maxVel, 0);

        rightFollower.configureEncoder(Robot.driveTrain.getFrontRightEncoder(), Constants.encoderTicksPerRotation,
                Constants.wheelDiameter);
        rightFollower.configurePIDVA(0.3, 0, 0, 1 / Constants.maxVel, 0);

        System.out.println("Starting path follow");
        followerNotifier = new Notifier(this::followPath);
        followerNotifier.startPeriodic(leftTrajectory.get(0).dt);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
        followerNotifier.stop();
        Robot.driveTrain.stop();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    public void followPath() {
        if (leftFollower.isFinished() || rightFollower.isFinished()) {
            followerNotifier.stop();
        } else {
            double left_speed = leftFollower.calculate(Robot.driveTrain.getFrontLeftEncoder());
            double right_speed = rightFollower.calculate(Robot.driveTrain.getFrontRightEncoder());
            // double heading = -Robot.driveTrain.getAngle();
            // double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
            // double heading_difference = Pathfinder.boundHalfDegrees(desired_heading -
            // heading);
            // double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
            // // System.out.println("Turn: " + turn);
            // System.out.println("Left speed: " + left_speed + " Combined: " + (left_speed
            // + turn));
            // System.out.println("Right speed: " + right_speed + " Combined: " +
            // (right_speed + turn));
            System.out.println("Left speed: " + left_speed);
            System.out.println("Right speed: " + right_speed);
            // Robot.driveTrain.tankDrive(left_speed + turn, right_speed - turn, false);
            Robot.driveTrain.tankDrive(left_speed, right_speed, false);
        }
    }
}
