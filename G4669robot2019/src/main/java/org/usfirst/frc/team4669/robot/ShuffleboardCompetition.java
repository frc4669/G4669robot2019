/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import java.util.Map;

import org.usfirst.frc.team4669.robot.commands.arm.StartElbow;
import org.usfirst.frc.team4669.robot.commands.arm.StartShoulder;
import org.usfirst.frc.team4669.robot.commands.arm.StartWrist;
import org.usfirst.frc.team4669.robot.commands.arm.StarterArm;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardCompetition {
    static ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
    private static NetworkTableEntry frontDist,rearDist, compStatus, pressureStatus,leftClimbSensor,rightClimbSensor, ballModeToggle, hatchModeToggle,calibrateMode;
    public static void initialize(){
		compTab.add("Gyro",(Sendable) Robot.driveTrain.getGyro()).withProperties(Map.of("Label position", "HIDDEN"));
		ShuffleboardLayout compressor = compTab.getLayout("Compressor", BuiltInLayouts.kList).withSize(2, 4);
		compStatus = compressor.add("Compressor Enabled", Robot.grabber.isCompressorRunning()).getEntry();
		pressureStatus = compressor.add("Pressure Low", Robot.grabber.isPressureLow()).getEntry();
		ShuffleboardLayout distanceSensors = compTab.getLayout("Distance", BuiltInLayouts.kGrid).withSize(2, 6);
		// frontDist = distanceSensors.add("Front Dist", Robot.driveTrain.getFrontDistance()).getEntry();
    // rearDist = distanceSensors.add("Rear Dist", Robot.driveTrain.getRearDistance()).getEntry();
    distanceSensors.add("Front Dist", Robot.driveTrain.frontUltrasonic).withWidget("Ultrasonic");
    distanceSensors.add("Rear Dist", Robot.driveTrain.rearUltrasonic).withWidget("Ultrasonic");
    leftClimbSensor = distanceSensors.add("Left Climb on Plat", Robot.elevator.getLeftSensor()>1.8).getEntry();
    rightClimbSensor = distanceSensors.add("Right Climb on Plat", Robot.elevator.getRightSensor()>1.8).getEntry();
    ballModeToggle = compTab.add("Ball Mode", Robot.toggleBallMode).getEntry();
    hatchModeToggle = compTab.add("Hatch Mode", !Robot.toggleBallMode).getEntry();
    calibrateMode = compTab.add("Calibrate Mode", false).getEntry();
    compTab.add("Start Arm Pos", new StarterArm());
    compTab.add("Start Shoulder", new StartShoulder());
    compTab.add("Start Elbow", new StartElbow());
    compTab.add("Start Wrist", new StartWrist());


    }

    public static void createAuto(SendableChooser chooser){
		compTab.add("Auto Mode", chooser).withSize(3, 1);
    }

    public static void update(){
      // frontDist.setDouble(Robot.driveTrain.getFrontDistance());
      // rearDist.setDouble(Robot.driveTrain.getRearDistance());
      compStatus.setBoolean(Robot.grabber.isCompressorRunning());
      pressureStatus.setBoolean(Robot.grabber.isPressureLow());
      leftClimbSensor.setBoolean(Robot.elevator.getLeftSensor()>1.8);
      rightClimbSensor.setBoolean(Robot.elevator.getRightSensor()>1.8);
      ballModeToggle.setBoolean(Robot.toggleBallMode);
      hatchModeToggle.setBoolean(!Robot.toggleBallMode);
      Robot.toggleCalibrate = calibrateMode.getBoolean(false);

    }
}
