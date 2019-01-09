package org.usfirst.frc.team4669.robot;

import edu.wpi.first.wpilibj.Joystick;

public class F310 {
	
	//x on back switch, mode light off
	private Joystick f310;
	
	public static final int greenButton = 1,
							redButton = 2,
							blueButton = 3,
							orangeButton = 4,
							leftShoulderButton = 5,
							rightShoulderButton = 6,
							backButton = 7,
							startButton = 8,
							leftJoyButton = 9,
							rightJoyButton = 10;
	
	public F310() {
		f310 = new Joystick(RobotMap.f310);
	}
	
	public double getLeftX() {
		return deadzone(0);
	}
	
	public double getLeftXSquared() {
		return Math.pow(getLeftX(),3);
	}
	
	public double getLeftY() {
		return -deadzone(1); //Negates the Y axis of the joystick so that up is positive and down is negative
	}
	
	public double getLeftYCubed() {
		return Math.pow(getLeftY(),3);
	}
	
	public double getLeftTrigger() {
		return deadzone(2);
	}
	
	public double getRightTrigger() {
		return deadzone(3);
	}
	
	public double getRightX() {
		return deadzone(4);
	}
	
	public double getRightXCubed() {
		return Math.pow(getRightX(),3);
	}
	
	//ControlWinch
	public double getRightY() {
		return -deadzone(5); //Negates the Y axis of the joystick so that up is positive and down is negative
	}
	
	public boolean getButton(int buttonPort) {
		return f310.getRawButton(buttonPort);
	}
	
	public boolean getButtonReleased(int buttonPort) {
		return f310.getRawButtonReleased(buttonPort);
	}
	
	
	public double getDPadPOV() {
		return f310.getPOV();
	}
	
	public double deadzone(int port){
		double joystickValue = f310.getRawAxis(port);
		double joystickOffset = 0.075;
		double absJoystickValue = Math.abs(joystickValue);
		if (absJoystickValue > joystickOffset) {
			double speed = absJoystickValue;
			speed = (speed*speed) + joystickOffset;
			if (joystickValue > 0) 
				return speed;
			else
				return -speed;
		}
		else {
			return 0;
		}
	}

}
