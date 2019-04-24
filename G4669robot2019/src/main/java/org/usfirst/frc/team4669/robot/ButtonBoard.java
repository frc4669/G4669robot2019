

package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.arm.ArmData;
import org.usfirst.frc.team4669.robot.commands.arm.PathCommand;
import org.usfirst.frc.team4669.robot.commands.arm.PositionCommand;
import org.usfirst.frc.team4669.robot.commands.arm.RetractArm;
import org.usfirst.frc.team4669.robot.commands.driveTrain.DriveForwardMotionMagic;
import org.usfirst.frc.team4669.robot.commands.grabber.ToggleCompressor;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ButtonBoard {

	private Joystick buttonBoard;
	public Button button1, button2, button4, button5, button6, button7,
	 button9, button10, button11, button12;
	

	public ButtonBoard(int portID) {
		buttonBoard = new Joystick(portID);
		button1 = new JoystickButton(buttonBoard, 1);
		button2 = new JoystickButton(buttonBoard, 2);
		
		button4 = new JoystickButton(buttonBoard, 4);
		button5 = new JoystickButton(buttonBoard, 5);
		button6 = new JoystickButton(buttonBoard, 6);
		button7 = new JoystickButton(buttonBoard, 7);
		
		button9 = new JoystickButton(buttonBoard, 9);
		button10 = new JoystickButton(buttonBoard, 10);
		button11 = new JoystickButton(buttonBoard, 11);
		button12 = new JoystickButton(buttonBoard, 12);
		
	}


	public boolean getButton(int buttonPort) {
		return buttonBoard.getRawButton(buttonPort);
	}

	public boolean getButtonReleased(int buttonPort) {
		return buttonBoard.getRawButtonReleased(buttonPort);
	}

	public boolean getButtonPressed(int buttonPort) {
		return buttonBoard.getRawButtonPressed(buttonPort);
	}

	public double getX() {
		return -buttonBoard.getRawAxis(0);
	}
	public double getY() {
		return buttonBoard.getRawAxis(1);
	}
	
	public boolean isRight() {
		return (getX() == 1);
	}
	
	public boolean isLeft() {
		return (getX() == -1);
	}
	
	public boolean isUp() {
		return (getY() == 1);
	}
	
	public boolean isDown() {
		return (getY() == -1);
	}
	
	public double getAngle() {
		if (isRight()) {
			return 0;
		}
		else if (isRight() && isUp()) {
			return 180-28.75;
		}
		else if (isUp()) {
			return 90;
		}
		else if (isLeft() && isUp()) {
			return 180+28.75;
		}
		else if (isLeft()) {
			return 180;
		}
		else if (isLeft() && isDown()) {
			return 360-28.75;
		}
		else if (isDown()) {
			return 270;
		}
		else if (isRight() && isDown()) {
			return 28.75;
		}
		else {
			return -1;
		}
	}
}
