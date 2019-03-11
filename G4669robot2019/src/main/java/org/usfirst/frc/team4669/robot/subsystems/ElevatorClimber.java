package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.AnalogDistanceSensor;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.elevator.TeleopClimber;
import org.usfirst.frc.team4669.robot.misc.Constants;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * Robot elevator climber system.
 */
public class ElevatorClimber extends Subsystem {

    private WPI_TalonSRX wheelMotor;
    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;
    private Accelerometer accel;

    private AnalogDistanceSensor leftSensor;
    private AnalogDistanceSensor rightSensor;


    private int timeout = Constants.timeout;
    private int slotIdx = RobotMap.slotIdx;
    private int pidIdx = RobotMap.pidIdx;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public ElevatorClimber() {
        super();
        leftMotor = new WPI_TalonSRX(RobotMap.leftMotorElevator);
        rightMotor = new WPI_TalonSRX(RobotMap.rightMotorElevator);
        wheelMotor = new WPI_TalonSRX(RobotMap.wheelMotorElevator);

        leftSensor = new AnalogDistanceSensor(RobotMap.leftInfrared);
        rightSensor = new AnalogDistanceSensor(RobotMap.rightInfrared);

        leftSensor.setScaleFactor(Constants.infraredScale);
        rightSensor.setScaleFactor(Constants.infraredScale);

        accel = new BuiltInAccelerometer();

        setupMotor(leftMotor, false, Constants.elevatorPID, Constants.elevatorVel, Constants.elevatorAccel, false, true);
        leftMotor.configForwardSoftLimitEnable(false);
        // leftMotor.configForwardSoftLimitThreshold(0);
        leftMotor.configReverseSoftLimitEnable(true);
        leftMotor.configReverseSoftLimitThreshold((int) (-Constants.limitMaxHeight * Constants.inchToEncoderElevator));

        setupMotor(rightMotor, false, Constants.elevatorPID, Constants.elevatorVel, Constants.elevatorAccel, false,
                true);
        rightMotor.configForwardSoftLimitEnable(false);
        // rightMotor.configForwardSoftLimitThreshold(0);
        rightMotor.configReverseSoftLimitEnable(true);
        rightMotor.configReverseSoftLimitThreshold((int) (-Constants.limitMaxHeight * Constants.inchToEncoderElevator));
        setupMotor(wheelMotor, true, Constants.elevatorWheelPID, Constants.elevatorWheelVel,
                Constants.elevatorWheelAccel, true, false);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new TeleopClimber());
    }

    public void percentOutputLeft(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    public void percentOutputRight(double percent) {
        rightMotor.set(ControlMode.PercentOutput, percent);
    }

    public void percentOutputWheel(double percent) {
        wheelMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
        wheelMotor.set(0);
    }

    public void setMagicVelAccel(TalonSRX talon, int velocity, int accel) {
        talon.configMotionCruiseVelocity(velocity);
        talon.configMotionAcceleration(accel);
    }

    public void setupMotor(TalonSRX talon, boolean invert, double[] pidArray, int motionMagicVel, int motionMagicAccel,
            boolean isWheel, boolean sensorPhase) {
        talon.configFactoryDefault();
        talon.configNominalOutputForward(0, timeout);
        talon.configNominalOutputReverse(0, timeout);
        talon.configPeakOutputForward(1, timeout);
        talon.configPeakOutputReverse(-1, timeout);

        // Sets current limits
        talon.configContinuousCurrentLimit(12, timeout);
        talon.configPeakCurrentLimit(15, timeout);
        talon.configPeakCurrentDuration(100, timeout);
        talon.enableCurrentLimit(true);

        // Sets up sensor
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeout);
        talon.setSensorPhase(sensorPhase);
        talon.setInverted(invert);
        talon.setSelectedSensorPosition(0, pidIdx, timeout);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.timeout);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.timeout);

        // Configuring PID Values
        talon.selectProfileSlot(slotIdx, pidIdx);
        talon.config_kF(slotIdx, pidArray[0], timeout);
        talon.config_kP(slotIdx, pidArray[1], timeout);
        talon.config_kI(slotIdx, pidArray[2], timeout);
        talon.config_kD(slotIdx, pidArray[3], timeout);
        talon.config_IntegralZone(slotIdx, (int) pidArray[4], timeout);

        // Set relevant frame periods to be at least as fast as periodic rate
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);

        // Sets up velocity and acceleration for motion magic
        talon.configMotionCruiseVelocity(motionMagicVel, Constants.timeout);
        talon.configMotionAcceleration(motionMagicAccel, Constants.timeout);

        if (isWheel) {
            // Zero encoders on fwd limit or bottom limit
            talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);
        }
    }

    public void setMotionMagic(TalonSRX talon, double position) {
        talon.set(ControlMode.MotionMagic, -position);
    }

    public int getEncoderPos(TalonSRX talon) {
        return talon.getSelectedSensorPosition();
    }

    public int getEncoderVel(TalonSRX talon) {
        return talon.getSelectedSensorVelocity();
    }

    public double getMotorOutput(TalonSRX talon) {
        return talon.getMotorOutputPercent();
    }

    public void zeroEncoders() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    public void zeroWheelEncoder() {
        wheelMotor.setSelectedSensorPosition(0);
    }

    public boolean getForwardLimit(TalonSRX talon) {
        return talon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimit(TalonSRX talon) {
        return talon.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void zeroVelocity(TalonSRX talon) {
        talon.set(ControlMode.Velocity, 0);
    }

    public double getLeftHeight(){
        return leftSensor.getDistance();
    }

    public double getRightHeight(){
        return leftSensor.getDistance();
    }

    public double getAccelX() {
        return accel.getX();
    }

    public double getAccelY() {
        return accel.getY();
    }

    public TalonSRX getLeftMotor() {
        return leftMotor;
    }

    public TalonSRX getRightMotor() {
        return rightMotor;
    }

    public TalonSRX getWheelMotor() {
        return wheelMotor;
    }

    public void setVelocity(TalonSRX talon, double velocity) {
        talon.set(ControlMode.Velocity, velocity);
    }
}
