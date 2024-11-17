package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ColorController {
    public enum ColorStatus{
        SEEING_RED,
        SEEING_BLUE,
        SEEING_YELLOW,
        SEEING_NOTHING
    }
    public ElapsedTime timer_red = new ElapsedTime();
    public ElapsedTime timer_blue = new ElapsedTime();
    public ElapsedTime timer_yellow = new ElapsedTime();
    public ElapsedTime timer_nothing = new ElapsedTime();
    public ColorStatus currentStatus = ColorStatus.SEEING_NOTHING;
    public ColorStatus previousStatus = null;
    public ColorSensor colorSensor = null;

    public ColorController(RobotMap robot)
    {
        this.colorSensor = robot.colorSensor;
    }
    public void update()
    {
        switch(currentStatus)
        {
            case SEEING_NOTHING:
            {
                timer_blue.reset();
                timer_red.reset();
                timer_yellow.reset();
                if(colorSensor.red()>400 && colorSensor.green()>400 && colorSensor.blue()<300)
                {
                    currentStatus=ColorStatus.SEEING_YELLOW;
                }
                else if(colorSensor.red() > 300 && colorSensor.blue()<300 && colorSensor.green()<300)
                {
                    currentStatus=ColorStatus.SEEING_RED;
                }
                else if(colorSensor.blue()>300 && colorSensor.red()<300 &&  colorSensor.green()<300)
                {
                    currentStatus = ColorStatus.SEEING_BLUE;
                }
                break;
            }
            case SEEING_RED:
            {
                timer_blue.reset();
                timer_nothing.reset();
                timer_yellow.reset();
                if(colorSensor.red()>400 && colorSensor.green()>400 && colorSensor.blue()<300)
                {
                    currentStatus=ColorStatus.SEEING_YELLOW;
                }
                else if(colorSensor.red() > 300 && colorSensor.blue()<300 && colorSensor.green()<300)
                {
                    currentStatus=ColorStatus.SEEING_RED;
                }
                else if(colorSensor.blue()>300 && colorSensor.red()<300 &&  colorSensor.green()<300)
                {
                    currentStatus = ColorStatus.SEEING_BLUE;
                }
                else{
                    currentStatus = ColorStatus.SEEING_NOTHING;
                }
                break;
            }
            case SEEING_BLUE:
            {
                timer_red.reset();
                timer_yellow.reset();
                timer_nothing.reset();
                if(colorSensor.red()>400 && colorSensor.green()>400 && colorSensor.blue()<300)
                {
                    currentStatus=ColorStatus.SEEING_YELLOW;
                }
                else if(colorSensor.red() > 300 && colorSensor.blue()<300 && colorSensor.green()<300)
                {
                    currentStatus=ColorStatus.SEEING_RED;
                }
                else if(colorSensor.blue()>300 && colorSensor.red()<300 &&  colorSensor.green()<300)
                {
                    currentStatus = ColorStatus.SEEING_BLUE;
                }
                else{
                    currentStatus = ColorStatus.SEEING_NOTHING;
                }
                break;
            }
            case SEEING_YELLOW:
            {
                timer_red.reset();
                timer_blue.reset();
                timer_nothing.reset();
                if(colorSensor.red()>400 && colorSensor.green()>400 && colorSensor.blue()<300)
                {
                    currentStatus=ColorStatus.SEEING_YELLOW;
                }
                else if(colorSensor.red() > 300 && colorSensor.blue()<300 && colorSensor.green()<300)
                {
                    currentStatus=ColorStatus.SEEING_RED;
                }
                else if(colorSensor.blue()>300 && colorSensor.red()<300 &&  colorSensor.green()<300)
                {
                    currentStatus = ColorStatus.SEEING_BLUE;
                }
                else{
                    currentStatus = ColorStatus.SEEING_NOTHING;
                }
                break;
            }
        }
    }
}
