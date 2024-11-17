package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ArmController {
    public enum ArmStatus
    {
        INIT,
        MID,
        HIGH,
        COLLECT_MAX,
        RUNTO
    }
    public ArmStatus currentStatus = ArmStatus.INIT;
    public ArmStatus previousStatus = null;
    public static int init_position = 0;
    public static int max_collect_position=300;
    public static int mid_position=1350;//1200
    public static int high_position = 1300;
    public int currentPosition = init_position;
    public DcMotorEx arm = null;
    public ArmController(RobotMap robot){this.arm = robot.arm;}
    public void update(int target)
    {
        this.arm.setTargetPosition(currentPosition);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(currentStatus!=ArmStatus.INIT) {
            this.arm.setPower(1);
        }
        double arm_current_position = arm.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==ArmStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus) {
                case INIT:
                {
                    this.arm.setPower(0.5);
                    currentPosition = init_position;
                    break;
                }
                case MID:
                {
                    currentPosition=mid_position;
                    break;
                }
                case HIGH:
                {
                 currentPosition = high_position;
                 break;
                }
                case RUNTO:
                {
                    currentPosition=target;
                    break;
                }
                case COLLECT_MAX:
                {
                    currentPosition=max_collect_position;
                    break;
                }
            }
        }
    }
}
