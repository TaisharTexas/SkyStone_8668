package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.sbfUtil.PVector;

import java.util.ArrayList;

public class Path
{
    ArrayList<PVector> pathPoints;
    ArrayList<Double> maxSpeeds;
    ArrayList<Double> targetHeadings;


    public Path()
    {
        pathPoints = new ArrayList<PVector>();
        maxSpeeds = new ArrayList<Double>();
        targetHeadings = new ArrayList<Double>();
    }

    public void addPoint(float x, float y, double maxSpeed, double desiredHeading)
    {
        PVector point = new PVector(x, y);
        pathPoints.add(point);
        maxSpeeds.add(maxSpeed);
        targetHeadings.add(desiredHeading);
    }
}
