package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class IntakeController {
    public enum IntakeStatus{
        STOP,
        REVERSE,
        COLLECT;
    }
    public IntakeStatus currentStatus= IntakeStatus.STOP;
    public IntakeStatus previousStatus= null;
    public double collectPower=-1f;
    public double collectPower2=1f;
    public double reversePower=0.13f;
    public double reversePower2=-0.13f;
    public double stopPower=0f;
    public CRServo intake1;
    public CRServo intake2;
    public IntakeController(RobotMap robot){
        this.intake1=robot.intake1;
        this.intake2=robot.intake2;
    }
    public void update(){
        if(currentStatus != previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case STOP:
                {
                    intake1.setPower(stopPower);
                    intake2.setPower(stopPower);
                    break;
                }
                case COLLECT:
                {
                    intake1.setPower(collectPower);
                    intake2.setPower(collectPower2);
                    break;
                }
                case REVERSE:
                {
                    intake1.setPower(reversePower);
                    intake2.setPower(reversePower2);
                    break;
                }
            }
        }
    }
}
