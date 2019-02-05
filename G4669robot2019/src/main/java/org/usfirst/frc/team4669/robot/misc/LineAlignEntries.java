package org.usfirst.frc.team4669.robot.misc;

import org.usfirst.frc.team4669.robot.Robot;

public class LineAlignEntries {
    private int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    private double angle = 0;
    private boolean front = true;

    public LineAlignEntries(boolean front) {
        updateEntries();
    }

    public void updateEntries() {
        if (front) {
            if (Robot.visionTable.containsKey("LineX0F"))
                x0 = Robot.visionTable.getEntry("LineX0F").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineY0F"))
                y0 = Robot.visionTable.getEntry("LineY0F").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineAngleF"))
                angle = Robot.visionTable.getEntry("LineAngleF").getNumber(0).doubleValue();
            if (Robot.visionTable.containsKey("LineX1F"))
                x1 = Robot.visionTable.getEntry("LineX1F").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineY1F"))
                y1 = Robot.visionTable.getEntry("LineY1F").getNumber(0).intValue();
        } else {
            if (Robot.visionTable.containsKey("LineX0R"))
                x0 = Robot.visionTable.getEntry("LineX0R").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineY0R"))
                y0 = Robot.visionTable.getEntry("LineY0R").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineAngleR"))
                angle = Robot.visionTable.getEntry("LineAngleR").getNumber(0).doubleValue();
            if (Robot.visionTable.containsKey("LineX1R"))
                x1 = Robot.visionTable.getEntry("LineX1R").getNumber(0).intValue();
            if (Robot.visionTable.containsKey("LineY1R"))
                y1 = Robot.visionTable.getEntry("LineY1R").getNumber(0).intValue();
        }
    }

    public int getX0() {
        updateEntries();
        return x0;
    }

    public int getY0() {
        updateEntries();
        return y0;
    }

    public int getX1() {
        updateEntries();
        return x1;
    }

    public int getY1() {
        updateEntries();
        return y1;
    }

    public double getAngle() {
        updateEntries();
        return angle;
    }

    public boolean isFrontFacing() {
        return front;
    }
}
