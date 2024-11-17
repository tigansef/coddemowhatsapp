package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class FourbarController {
    public enum FourbarStatus{
        INIT,
        COLLECT,
        PLACESPECIMEN,
        RUNTO;
    }
    public FourbarStatus currentStatus = FourbarStatus.INIT;
    public FourbarStatus previousStatus=null;
    public static double init_position=0;
    public static double collect_position = 0.95;
    public static double specimen_position = 1; //0.7
    public Servo fourbar = null;

        public FourbarController(RobotMap robot) {
        this.fourbar=robot.fourbar;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == FourbarStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.fourbar.setPosition(init_position);
                    break;
                }
                case COLLECT:
                {
                    this.fourbar.setPosition(collect_position);
                    break;
                }
                case PLACESPECIMEN:
                {
                    this.fourbar.setPosition(specimen_position);
                    break;
                }
            }
        }
    }
}
