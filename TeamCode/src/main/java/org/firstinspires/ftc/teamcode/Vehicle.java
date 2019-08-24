package org.firstinspires.ftc.teamcode;

public class Vehicle
{
    public PVector location;
    public PVector velocty;
    public PVector accleration;
    double endZone;
    double maxSpeed;
    double maxAcceleration;

    Vehicle(float x, float y)
    {
        accleration = new PVector(0,0);
        velocty = new PVector(0,0);
        location = new PVector(x,y);
        endZone = 6; //inch
        maxSpeed = 0.75;
        maxAcceleration = 0.3;

    }

    public void arrive(PVector target)
    {
        PVector desiredVelocity = PVector.sub(target, location);
        float speed = desiredVelocity.mag();

        if(speed < endZone)
        {
            float m = vectorScale(speed, 0, (float)endZone, 0, (float)maxSpeed);
            desiredVelocity.setMag(m);
        }
        else
        {
            desiredVelocity.setMag(maxSpeed);
        }

        PVector steerAcceleration = PVector.sub(desiredVelocity, velocty);
        steerAcceleration.limit(maxAcceleration);

        velocty.add(steerAcceleration);
        velocty.limit(maxSpeed);

//        return velocty;

        //need to send steer force to motors...somehow
    }

    public float vectorScale(float value, float start1, float stop1, float start2, float stop2)
    {
        float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));

        return outgoing;
    }


}
