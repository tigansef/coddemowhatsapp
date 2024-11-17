package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ExtenderController {
    public enum ExtenderStatus
    {
        INIT,
        FAR,
        COLLECT_FAR,
        SCORE,
        RUNTO;
    }
    public ExtenderStatus currentStatus = ExtenderStatus.INIT;
    public ExtenderStatus previousStatus = null;
    public static int init_position = 0;
    public static int far_position = 500;//1650
    //max 700
    public static int score_position = -100;
    public static int collect_far_position = 200;//1000
    public int currentPosition = init_position;
    public DcMotorEx extender = null;
    public ExtenderController(RobotMap robot)
    {
        this.extender = robot.extender;
    }

    public void update(int targetPosition)
    {
        this.extender.setTargetPosition(currentPosition);
        this.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.extender.setPower(1);
        double extender_current_position = extender.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==ExtenderStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    currentPosition=init_position;
                    targetPosition=0;
                    break;
                }
                case FAR:
                {
                    currentPosition=far_position;
                    break;
                }
                case RUNTO:
                {
                    currentPosition=targetPosition;
                    break;
                }
                case SCORE:
                {
                    currentPosition=score_position;
                    break;
                }
                case COLLECT_FAR:
                {
                    currentPosition=collect_far_position;
                    break;
                }
            }
        }
    }

}
