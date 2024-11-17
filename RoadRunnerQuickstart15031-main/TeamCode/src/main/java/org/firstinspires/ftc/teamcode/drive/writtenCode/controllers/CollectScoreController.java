package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class CollectScoreController {
    public enum CollectScoreStatus{
        INIT,
        WAIT_FOR_COLLECT,
        WAIT_FOR_SCORE,
        COLLECT,
        SCORE
    }
    public CollectScoreStatus currentStatus = CollectScoreStatus.INIT;
    public CollectScoreStatus previousStatus = null;
    private ExtenderController extenderController = null;
    private ArmController armController = null;
    private FourbarController fourbarController = null;
    private ClawPositionController clawPositionController = null;
    public double time_between = 0.5;
    public double time_between_score = 0.7;
    public ElapsedTime timer_interval = new ElapsedTime();
    public CollectScoreController(ArmController armController,
                                  ExtenderController extenderController,
                                  FourbarController fourbarController,
                                  ClawPositionController clawPositionController,
                                  RobotMap robot)
    {
        this.armController= armController;
        this.extenderController=extenderController;
        this.clawPositionController = clawPositionController;
        this.fourbarController = fourbarController;
    };
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus==CollectScoreStatus.WAIT_FOR_COLLECT || currentStatus==CollectScoreStatus.WAIT_FOR_SCORE)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    armController.currentStatus= ArmController.ArmStatus.INIT;
                    extenderController.currentStatus= ExtenderController.ExtenderStatus.INIT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                    fourbarController.currentStatus = FourbarController.FourbarStatus.INIT;

                    break;
                }
                case COLLECT:
                {
                    armController.currentStatus= ArmController.ArmStatus.COLLECT_MAX;
                    fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT;
                    timer_interval.reset();
                    currentStatus=CollectScoreStatus.WAIT_FOR_COLLECT;
                    break;
                }
                case WAIT_FOR_COLLECT:
                {
                    if(timer_interval.seconds()>=time_between)
                    {
                        extenderController.currentStatus=ExtenderController.ExtenderStatus.FAR;

                    }


                    break;
                }
                case SCORE:
                {
                    armController.currentStatus= ArmController.ArmStatus.HIGH;
                    timer_interval.reset();
                    currentStatus=CollectScoreStatus.WAIT_FOR_SCORE;
                    break;
                }
                case WAIT_FOR_SCORE:
                {
                    if(timer_interval.seconds()>=time_between_score)
                    {
                        //extenderController.currentStatus= ExtenderController.ExtenderStatus.SCORE;
                    }
                    break;
                }
            }
        }
    }

}
