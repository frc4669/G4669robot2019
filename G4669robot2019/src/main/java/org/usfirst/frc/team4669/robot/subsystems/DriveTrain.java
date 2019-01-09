/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.MecanumDriveCommand;
import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.misc.PIDOutputWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX frontLeftMotor;
  private WPI_TalonSRX rearLeftMotor;
  private WPI_TalonSRX frontRightMotor;
  private WPI_TalonSRX rearRightMotor;

  private SpeedControllerGroup leftMotorGroup;
  private SpeedControllerGroup rightMotorGroup;
  private MecanumDrive drive;

  public PIDController gyroPID;
  private PIDOutputWrapper turnOutput;
  private Gyro analogGyro;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MecanumDriveCommand());
    // setDefaultCommand(new MySpecialCommand());
  }

  public DriveTrain() {
    // 0 for placeholder
    frontLeftMotor = new WPI_TalonSRX(0);
    rearLeftMotor = new WPI_TalonSRX(0);
    frontRightMotor = new WPI_TalonSRX(0);
    rearRightMotor = new WPI_TalonSRX(0);

    analogGyro = new ADXRS450_Gyro();

    drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  }

  /**
   * Cartesian drive method that specifies speeds in terms of the field
   * longitudinal and lateral directions, using the drive's angle sensor to
   * automatically determine the robot's orientation relative to the field.
   * <p>
   * Using this method, the robot will move away from the drivers when the
   * joystick is pushed forwards, and towards the drivers when it is pulled
   * towards them - regardless of what direction the robot is facing.
   * 
   * @param ySpeed        The speed that the robot should drive in the X
   *                      direction. [-1.0..1.0]
   * @param xSpeed        The speed that the robot should drive in the Y
   *                      direction. This input is inverted to match the forward
   *                      == -1.0 that joysticks produce. [-1.0..1.0]
   * @param rotationAngle The rate of rotation for the robot that is completely
   *                      independent of the translation. [-1.0..1.0]
   * @param gyroAngle     The angle of the robot relative to the field in degrees
   */
  public void fieldOrientedDrive(double ySpeed, double xSpeed, double rotationAngle, double gyroAngle) {
    drive.driveCartesian(ySpeed, xSpeed, rotationAngle, gyroAngle);
  }

  /**
   * Cartesian drive method that specifies speeds in terms of the field
   * longitudinal and lateral directions, using the drive's angle sensor to
   * automatically determine the robot's orientation relative to the field.
   * <p>
   * Using this method, the robot will move away from the drivers when the
   * joystick is pushed forwards, and towards the drivers when it is pulled
   * towards them - regardless of what direction the robot is facing.
   * 
   * @param ySpeed        The speed that the robot should drive in the X
   *                      direction. [-1.0..1.0]
   * @param xSpeed        The speed that the robot should drive in the Y
   *                      direction. This input is inverted to match the forward
   *                      == -1.0 that joysticks produce. [-1.0..1.0]
   * @param rotationAngle The rate of rotation for the robot that is completely
   *                      independent of the translation. [-1.0..1.0]
   */
  public void robotOrientedDrive(double ySpeed, double xSpeed, double rotationAngle) {
    drive.driveCartesian(ySpeed, xSpeed, rotationAngle);
  }

  /**
   * Polar drive method that specifies speeds in terms of magnitude and direction.
   * This method does not use the drive's angle sensor.
   * 
   * @param magnitude The speed that the robot should drive in a given direction.
   * @param direction The direction the robot should drive in degrees. The
   *                  direction and magnitude are independent of the rotation
   *                  rate.
   * @param rotation  The rate of rotation for the robot that is completely
   *                  independent of the magnitude or direction. [-1.0..1.0]
   */
  public void polarDrive(double magnitude, double direction, double rotation) {
    drive.drivePolar(magnitude, direction, rotation);
  }

  public void calibrateGyro() {
    analogGyro.calibrate();
  }

  public void resetGyro() {
    analogGyro.reset();
  }

  public double getGyroAngle() {
    return analogGyro.getAngle();
  }

  public int getFrontLeftEncoder() {
    return frontLeftMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getFrontRightEncoder() {
    return frontRightMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getRearLeftEncoder() {
    return rearLeftMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getRearRightEncoder() {
    return rearRightMotor.getSensorCollection().getQuadraturePosition();
  }

  public double getFrontLeftEncoderSpeed() {
    return frontLeftMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getFrontRightEncoderSpeed() {
    return frontRightMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getRearLeftEncoderSpeed() {
    return rearLeftMotor.getSensorCollection().getQuadratureVelocity();
  }

  public double getRearRightEncoderSpeed() {
    return rearRightMotor.getSensorCollection().getQuadratureVelocity();
  }

  public void zeroEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
    rearLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
    frontRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
    rearRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
  }

  public void stop() {
    drive.driveCartesian(0, 0, 0);
  }

}
