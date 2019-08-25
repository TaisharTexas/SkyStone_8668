package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vehicle
{
    public PVector location;
    public PVector velocity;
    public PVector acceleration;
    public PVector desiredVelocity;

    double endZone;
    double maxSpeed;
    double gain;

    Telemetry telemetry;

    Vehicle(float x, float y, Telemetry telem)
    {
        telemetry = telem;
        acceleration = new PVector(0, 0);
        velocity = new PVector(0,0);
        location = new PVector(x,y);
        desiredVelocity = new PVector(0,0);
        endZone = 6; //inch
        gain = 0.5;

        // 30 in/sec correlates to maximum unloaded motor speed //
        maxSpeed = 30; //inches/second
        //30.615 in/sec = 15.7 rad/sec * 1.3 gearing * 1.5 in wheel radius


    }

    public void arrive(PVector target)
    {
        //find the needed velocity to move to target and call it desiredVelocity
        desiredVelocity = PVector.sub(target, location);

        telemetry.addData("v.desired velocity: ", desiredVelocity);

        //speed is the magnitude of desiredVelocity
        float speed = desiredVelocity.mag();

        if(speed < endZone)
        {
            float m = vectorScale(speed, 0, (float)endZone, 0, (float)maxSpeed);
            desiredVelocity.setMag(m);
        }
        else
        {
            //set speed to maximum allowed speed
            desiredVelocity.setMag(maxSpeed);
        }

        telemetry.addData("v.desired velocity set maxSpd: ", desiredVelocity);


        // steerAcceleration is the amount of needed change in velocity
        PVector steerAcceleration = PVector.sub(desiredVelocity, velocity);
        telemetry.addData("v.steerAcceleration: ", steerAcceleration);

        // limit rate of change to robot velocity
        steerAcceleration.limit(gain);

        telemetry.addData("v.limit steerAcceleration: ", steerAcceleration);


        //corrects robot velocity by adding error (steerAcceleration)
        desiredVelocity = PVector.add(velocity, steerAcceleration);

        //make sure final velocity isn't too fast
        desiredVelocity.limit(maxSpeed);

        telemetry.addData("v.desired velocity: ", desiredVelocity);

    }

    public float vectorScale(float value, float start1, float stop1, float start2, float stop2)
    {
        float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));

        return outgoing;
    }


}
