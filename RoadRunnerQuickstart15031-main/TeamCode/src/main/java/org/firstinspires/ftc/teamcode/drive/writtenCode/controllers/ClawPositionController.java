package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawPositionController {
    public enum ClawPositionStatus{
        INIT,
        COLLECT,
        SPECIMEN,
        RUNTO;
    }
    public ClawPositionStatus currentStatus = ClawPositionStatus.INIT;
    public ClawPositionStatus previousStatus=null;
    public static double init_position=0;
    public static double collect_position = 0.2;
    public static double specimen_position = 0;
    public Servo clawPosition = null;
    public ClawPositionController(RobotMap robot) {
        this.clawPosition=robot.clawPosition;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == ClawPositionStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.clawPosition.setPosition(init_position);
                    break;
                }
                case COLLECT:
                {
                    this.clawPosition.setPosition(collect_position);
                    break;
                }
                case SPECIMEN:
                {
                    this.clawPosition.setPosition(specimen_position);
                    break;
                }
            }
        }
    }
}
