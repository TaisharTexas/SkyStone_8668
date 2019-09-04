package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Path
{
    ArrayList<PVector> pathPoints;
    ArrayList<Double> maxSpeeds;


    Path()
    {
        pathPoints = new ArrayList<PVector>();
        maxSpeeds = new ArrayList<Double>();
    }

    public void addPoint(float x, float y, double maxSpeed)
    {
        PVector point = new PVector(x, y);
        pathPoints.add(point);
        maxSpeeds.add(maxSpeed);
    }
}
