package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawController {
    public enum ClawStatus{
        OPEN,
        CLOSE;
    }
    public ClawController.ClawStatus currentStatus = ClawController.ClawStatus.CLOSE;
    public ClawController.ClawStatus previousStatus=null;
    public static double claw_right_open=0.1;
    public static double claw_left_open=0.9;
    public static double claw_right_closed=0.4;
    public static double claw_left_closed=0.6;
    public Servo rightClaw=null;
    public Servo leftClaw=null;

    public ClawController(RobotMap robot){
        this.leftClaw = robot.leftClaw;
        this.rightClaw = robot.rightClaw;
    }
    public void update(){
        if(currentStatus!=previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case OPEN:
                {
                    this.leftClaw.setPosition(claw_left_open);
                    this.rightClaw.setPosition(claw_right_open);
                    break;
                }
                case CLOSE:
                {
                    this.leftClaw.setPosition(claw_left_closed);
                    this.rightClaw.setPosition(claw_right_closed);
                    break;
                }
            }
        }
    }
}
