package org.usfirst.frc.team4669.robot.misc;

import org.usfirst.frc.team4669.robot.Robot;

public class VisionEntries {
    private int x = 0, y = 0, width = 0, height = 0, horizontalAngle = 0, distance = 0;
    private boolean isBallDetected = false;

    public VisionEntries() {
        updateEntries();
    }

    public void updateEntries() {
        isBallDetected = Robot.visionTable.getEntry("isBallDetected").getBoolean(false);
        if (Robot.visionTable.containsKey("orangeX"))
            x = Robot.visionTable.getEntry("orangeX").getNumber(0).intValue();
        else
            x = 0;
        if (Robot.visionTable.containsKey("orangeY"))
            y = Robot.visionTable.getEntry("orangeY").getNumber(0).intValue();
        else
            y = 0;
        if (Robot.visionTable.containsKey("orangeWidth"))
            width = Robot.visionTable.getEntry("orangeWidth").getNumber(0).intValue();
        else
            width = 0;
        if (Robot.visionTable.containsKey("orangeHeight"))
            height = Robot.visionTable.getEntry("orangeHeight").getNumber(0).intValue();
        else
            height = 0;
        if (Robot.visionTable.containsKey("horizontalAngle"))
            horizontalAngle = Robot.visionTable.getEntry("horizontalAngle").getNumber(0).intValue();
        else
            horizontalAngle = 0;
        if (Robot.visionTable.containsKey("distance"))
            distance = Robot.visionTable.getEntry("distance").getNumber(0).intValue();
        else
            distance = 0;
    }

    public int getX() {
        updateEntries();
        return x;
    }

    public int getY() {
        updateEntries();
        return y;
    }

    public int getWidth() {
        updateEntries();
        return width;
    }

    public int getHeight() {
        updateEntries();
        return height;
    }

    public double getHorizontalAngle() {
        updateEntries();
        return horizontalAngle;
    }

    public double getDistanceIfCentered() {
        updateEntries();
        return distance;
    }

    public boolean isObjectDetected() {
        updateEntries();
        return isBallDetected;
    }
}
