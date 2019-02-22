/**@author Khanh Hoang*/

package org.usfirst.frc.team4669.robot.pixy;

import org.usfirst.frc.team4669.robot.pixy.Pixy.ExposureSetting;
import org.usfirst.frc.team4669.robot.pixy.Pixy.WhiteBalanceSetting;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PixyTune{
    Pixy pixy;
    public PixyTune(Pixy pixy){
        this.pixy = pixy;
    }

    public void tunePixy(){
        SmartDashboard.putData("Pixy/GetParameters", new InstantCommand() {
        	{
        		setRunWhenDisabled(true);
        	}
        	@Override
        	protected void initialize() {
        		pixy.stopBlockProgram();
				pixy.stopFrameGrabber();
				
				getParameters();
				
        		pixy.startBlockProgram();
				pixy.startFrameGrabber();
        	}
        });
        SmartDashboard.putData("Pixy/SetParameters", new InstantCommand() {
        	{
        		setRunWhenDisabled(true);
        	}
        	@Override
        	protected void initialize() {
        		pixy.stopBlockProgram();
				pixy.stopFrameGrabber();
				
				setParameters();
				
        		//getParameters();
        		pixy.startBlockProgram();
				pixy.startFrameGrabber();
				
        	}
        });
    }

    private void getParameters() {
		boolean aec = pixy.getAutoExposure();
		boolean awb = pixy.getAutoWhiteBalance();
		ExposureSetting exp = pixy.getExposureCompensation();
		WhiteBalanceSetting wbv = pixy.getWhiteBalanceValue();
		SmartDashboard.putBoolean("Pixy/AutoExposure", aec);
		SmartDashboard.putBoolean("Pixy/AutoWhiteBalance", awb);
		SmartDashboard.putNumber("Pixy/ExposureGain", exp.gain);
		SmartDashboard.putNumber("Pixy/ExposureCompensation", exp.compensation);
		SmartDashboard.putNumber("Pixy/WhiteBalanceRed", wbv.red);
		SmartDashboard.putNumber("Pixy/WhiteBalanceGreen", wbv.green);
		SmartDashboard.putNumber("Pixy/WhiteBalanceBlue", wbv.blue);
		
    }

    private void setParameters() {
		boolean aec = SmartDashboard.getBoolean("Pixy/AutoExposure", false);
		boolean awb = SmartDashboard.getBoolean("Pixy/AutoWhiteBalance", false);
		int gain = (int) SmartDashboard.getNumber("Pixy/ExposureGain", 20);
		int comp = (int) SmartDashboard.getNumber("Pixy/ExposureCompensation", 100);
		int r = (int) SmartDashboard.getNumber("Pixy/WhiteBalanceRed", 64);
		int g = (int) SmartDashboard.getNumber("Pixy/WhiteBalanceGreen", 64);
		int b = (int) SmartDashboard.getNumber("Pixy/WhiteBalanceBlue", 64);
		pixy.setAutoExposure(aec);
		if (!aec) {
			pixy.setExposureCompensation(new ExposureSetting(gain, comp));
		}
		pixy.setAutoWhiteBalance(awb);
		if (!awb) {
			pixy.setWhiteBalanceValue(new WhiteBalanceSetting(r, g, b));
		}
	
	}
    
    public static void enumeratePixy(){
        SmartDashboard.putData("Pixy/Enumerate", new InstantCommand() {
        	{
        		setRunWhenDisabled(true);
        	}
			@Override
			protected void initialize() {
				Pixy.enumerate();
			}
		});
    }

    public void getPixyParams(){
        SmartDashboard.putData("Pixy/GetParameters", new InstantCommand() {
        	{
        		setRunWhenDisabled(true);
        	}
        	@Override
        	protected void initialize() {
        		pixy.stopBlockProgram();
				pixy.stopFrameGrabber();
				
				getParameters();
				
        		pixy.startBlockProgram();
				pixy.startFrameGrabber();
        	}
        });
    }
}