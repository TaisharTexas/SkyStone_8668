package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Path
{
    ArrayList<PVector> pathPoints;
    public double radius;


    Path()
    {
        radius = 20;
        pathPoints = new ArrayList<PVector>();
    }

    public void addPoint(float x, float y)
    {
        PVector point = new PVector(x, y);
        pathPoints.add(point);
    }
}
