package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vehicle
{
    public PVector location;
    public PVector velocity;
    public PVector accleration;
    double endZone;
    double maxSpeed;
    double gain;

    Telemetry telemetry;

    Vehicle(float x, float y, Telemetry telem)
    {
        telemetry = telem;
        accleration = new PVector(0,0);
        velocity = new PVector(0,0);
        location = new PVector(x,y);
        endZone = 6; //inch
        gain = 0.3;

        // 30 in/sec correlates to maximum unloaded motor speed //
        maxSpeed = 30; //inches/second
        //30.615 in/sec = 15.7 rad/sec * 1.3 gearing * 1.5 in wheel radius


    }

    public void arrive(PVector target)
    {
        telemetry.addData("starting location: ",location);
        telemetry.addData("starting velocity: ",velocity);

        //find the needed velocity to move to target and call it desiredVelocity
        PVector desiredVelocity = PVector.sub(target, location);

        telemetry.addData("desired velocity: ", desiredVelocity);

        //speed is the magnitude of desiredVelocity
        float speed = desiredVelocity.mag();

        telemetry.addData("speed: ", speed);

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

        telemetry.addData("desired velocity set maxSpd: ", desiredVelocity);

        telemetry.addData("the velocity: ",velocity);

        // steerAcceleration is the amount of needed change in velocity
        PVector steerAcceleration = PVector.sub(desiredVelocity, velocity);

        telemetry.addData("steerAcceleration: ", steerAcceleration);

        telemetry.addData("the velocity 2: ",velocity);

        // limit rate of change to robot velocity
        steerAcceleration.limit(gain);

        telemetry.addData("limit steerAcceleration: ", steerAcceleration);


        //corrects robot velocity by adding error (steerAcceleration)
        velocity.add(steerAcceleration);

        telemetry.addData("velocity with Accel: ", velocity);
        //make sure final velocity isn't too fast
        velocity.limit(maxSpeed);

        telemetry.addData("limited velocity: ", velocity);

    }

    public float vectorScale(float value, float start1, float stop1, float start2, float stop2)
    {
        float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));

        return outgoing;
    }


}
