package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class TuretaController {
    public enum TuretaStatus
    {
        INIT,
        RUNTO;
    }
    public TuretaStatus currentStatus = TuretaStatus.INIT;
    public TuretaController.TuretaStatus previousStatus = null;
    public int currentPosition = init_position;
    public static int init_position = 0;
    public static int left1_position = 400;
    public static int left2_position = 800;
    public static int left3_position = 1200;
    public DcMotorEx turret = null;
    public TuretaController(RobotMap robot) {
        this.turret=robot.turret;
    }
    public void update(int targetPosition)
    {
        this.turret.setTargetPosition(currentPosition);
        this.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turret.setPower(1);
        double extender_current_position = turret.getCurrentPosition();
        if(currentStatus!=previousStatus ){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT: {
                    currentPosition = init_position;
                    break;
                }
                case RUNTO:
                {
                    currentPosition = targetPosition;
                    break;
                }
            }
        }
    }
}
