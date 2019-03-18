/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands.arm;

import java.util.ArrayList;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmData {

    public static ArmData   
                            hatch1F = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.hatch1Height, 8, 0, -8, false),
                            hatch2F = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.hatch2Height, 5, 0, 0, false),
                            hatch1R = new ArmData(-(Constants.robotToArmBack + 24), 0, Constants.hatch1Height, 0, 180, 0, true),
                            hatch2R = new ArmData(-(Constants.robotToArmBack + Constants.xDistanceToPlace), 0, Constants.hatch2Height, 0, 180, 0, true),
                            hatch3R = new ArmData(-(Constants.robotToArmBack + 7), 0, Constants.hatch3Height, 0, 180, 0, true),
                            hookStart = new ArmData(Constants.robotToArmFront + 7, -3, 14, -1.5, -80, 80, false),
                            ballStart = new ArmData(Constants.robotToArmFront + 7, -3, 14, -1.5, -80, 60, false),
                            // start = new ArmData(Constants.robotToArmFront + 7, -3, 14, -1.5, -80, 60, false),
                            ballPickup = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace+3, 0, 7, 0, 0, 0, false),                     
                            ball1F = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.ball1Height, 0, 0, 0, false),
                            ballShip = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace+3, 0, Constants.ballShipHeight, 0, 0, 0, false),                     
                            ball2F = new ArmData(Constants.robotToArmFront + Constants.xDistanceToPlace, 0, Constants.ball2Height, 0, 0, 0, false),
                            ball1R = new ArmData(-(Constants.robotToArmBack + Constants.xDistanceToPlace), 0, Constants.ball1Height, 0, 180, 0, true),
                            ball2R = new ArmData(-(Constants.robotToArmBack + Constants.xDistanceToPlace), 0, Constants.ball2Height, 0, 180, 0, true),
                            ball3R = new ArmData(-(Constants.robotToArmBack + 10), 0, Constants.ball3Height, 0, 180, 0, true);
    
    public static ArrayList<ArmData> hatchList = null;
    public static ArrayList<ArmData> ballList = null;

    
    double x,  xCorrect,  y, yCorrect, targetGrabberAngle, angleCorrect;
    boolean flipUp;
    public ArmData(double x, double xCorrect, double y, double yCorrect, double targetGrabberAngle, double angleCorrect,
            boolean flipUp) {
        this.x = x; this.xCorrect = xCorrect; this.y = y; this.yCorrect=yCorrect; this.targetGrabberAngle = targetGrabberAngle; this.angleCorrect = angleCorrect; this.flipUp = flipUp;
    }

    public static void initData(){
        if(hatchList==null){
            hatchList = new ArrayList<ArmData>();
            hatchList.add(hookStart);
            hatchList.add(hatch1F);
            hatchList.add(hatch2F);
            hatchList.add(hatch3R);
            hatchList.add(hatch2R);
            hatchList.add(hatch1R);
        }
        if(ballList==null){
            ballList = new ArrayList<ArmData>();
            ballList.add(ballPickup);
            ballList.add(ballStart);
            ballList.add(ball1F);
            ballList.add(ballShip);
            ballList.add(ball2F);
            ballList.add(ball3R);
            ballList.add(ball2R);
            ballList.add(ball1R);
        }
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getGrabberAngle(){
        return targetGrabberAngle;
    }

    public boolean getFlip(){
        return flipUp;
    }
}
