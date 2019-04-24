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
import org.usfirst.frc.team4669.robot.commands.elevator.HomeElevators;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardCompetition {
    static ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
    private static NetworkTableEntry leftDist,rightDist, midDist, compStatus, pressureStatus,leftClimbSensor,midClimbSensor,rightClimbSensor, ballModeToggle, hatchModeToggle,calibrateMode;
    public static void initialize(){
		compTab.add("Gyro",(Sendable) Robot.driveTrain.getGyro()).withSize(2, 2).withPosition(7, 0).withProperties(Map.of("Label position", "HIDDEN"));
		ShuffleboardLayout compressor = compTab.getLayout("Compressor", BuiltInLayouts.kList).withSize(1, 2).withPosition(9, 0);
		compStatus = compressor.add("Compressor Enabled", Robot.grabber.isCompressorRunning()).getEntry();
		pressureStatus = compressor.add("Pressure Low", Robot.grabber.isPressureLow()).getEntry();
		ShuffleboardLayout distanceSensors = compTab.getLayout("Distance", BuiltInLayouts.kGrid).withSize(3, 2).withPosition(2, 0);
		// frontDist = distanceSensors.add("Front Dist", Robot.driveTrain.getFrontDistance()).withWidget("Ultrasonic").withSize(1, 1).withPosition(0, 0).getEntry();
    // rearDist = distanceSensors.add("Rear Dist", Robot.driveTrain.getRearDistance()).withWidget("Ultrasonic").getEntry();
    leftClimbSensor = distanceSensors.add("Left Climb on Plat", Robot.elevator.getLeftSensor()<Constants.climberLeftSensorTol).withSize(1, 1).withPosition(0, 1).getEntry();
    midClimbSensor = distanceSensors.add("Mid Climb on Plat", Robot.elevator.getMidSensor()<Constants.climberMidSensorTol).withSize(1, 1).withPosition(0, 1).getEntry();
    rightClimbSensor = distanceSensors.add("Right Climb on Plat", Robot.elevator.getRightSensor()<Constants.climberRightSensorTol).withSize(1, 1).withPosition(1, 1).getEntry();
    ShuffleboardLayout hatchBallLayout = compTab.getLayout("Hatch Ball", BuiltInLayouts.kGrid).withSize(1, 2).withPosition(6, 0);
    ballModeToggle = hatchBallLayout.add("Ball Mode", Robot.toggleBallMode).withSize(1, 1).withPosition(0, 0).getEntry();
    hatchModeToggle = hatchBallLayout.add("Hatch Mode", !Robot.toggleBallMode).withSize(1, 1).withPosition(0, 1).getEntry();
    ShuffleboardLayout calibration = compTab.getLayout("Calibration", BuiltInLayouts.kList).withSize(1, 2).withPosition(5, 0).withProperties(Map.of("Label position", "HIDDEN"));
    calibrateMode = calibration.add("Calibrate Mode", false).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    calibration.add("Start Arm Pos", new StarterArm());
    calibration.add("Start Shoulder", new StartShoulder());
    calibration.add("Start Elbow", new StartElbow());
    calibration.add("Start Wrist", new StartWrist());
    leftDist = distanceSensors.add("Left Dist", Robot.elevator.getLeftSensor()).withSize(1, 1).withPosition(0, 0).getEntry();
    midDist = distanceSensors.add("Mid Dist", Robot.elevator.getMidSensor()).withSize(1, 1).withPosition(0, 0).getEntry();
    rightDist = distanceSensors.add("Right Dist", Robot.elevator.getRightSensor()).withSize(1, 1).withPosition(1, 0).getEntry();
    compTab.add("Home Elevators", new HomeElevators()).withSize(1, 1).withPosition(0, 1);


    }

    public static void createAuto(SendableChooser chooser){
		compTab.add("Auto Mode", chooser).withSize(2, 1).withPosition(0, 0);
    }

    public static void update(){
      // frontDist.setDouble(Robot.driveTrain.getFrontDistance());
      // rearDist.setDouble(Robot.driveTrain.getRearDistance());
      compStatus.setBoolean(Robot.grabber.isCompressorRunning());
      pressureStatus.setBoolean(Robot.grabber.isPressureLow());
      leftClimbSensor.setBoolean(Robot.elevator.getLeftSensorUpdate()<Constants.climberLeftSensorTol);
      midClimbSensor.setBoolean(Robot.elevator.getMidSensor()<Constants.climberMidSensorTol);
      rightClimbSensor.setBoolean(Robot.elevator.getRightSensor()<Constants.climberRightSensorTol);
      ballModeToggle.setBoolean(Robot.toggleBallMode);
      hatchModeToggle.setBoolean(!Robot.toggleBallMode);
      Robot.toggleCalibrate = calibrateMode.getBoolean(false);
      leftDist.setNumber(Robot.elevator.getLeftSensor());
      midDist.setNumber(Robot.elevator.getMidSensor());
      rightDist.setNumber(Robot.elevator.getRightSensor());

    }

    public static void updateFront(){
      // frontDist.setNumber(Robot.driveTrain.getFrontDistance());
    }

    public static void updateRear(){
      // rearDist.setNumber(Robot.driveTrain.getRearDistance());
    }
}
