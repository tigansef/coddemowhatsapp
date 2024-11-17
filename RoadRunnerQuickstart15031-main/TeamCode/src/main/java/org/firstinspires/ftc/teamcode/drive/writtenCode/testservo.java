package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawPositionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawRotateController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectScoreController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ColorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.FourbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;

@TeleOp(name="testservo",group="Linear OpMode")
public class testservo extends LinearOpMode {
    public static double intakePower=0.1;
    int extenderTargetPosition = 0;
    int armTargetPosition = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RobotMap robot = new RobotMap(hardwareMap);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        ClawPositionController clawPositionController = new ClawPositionController(robot);
        ClawRotateController clawRotateController = new ClawRotateController(robot);
        ClawController clawController = new ClawController(robot);
        FourbarController fourbarController = new FourbarController(robot);
        IntakeController intakeController = new IntakeController(robot);
        ExtenderController extenderController = new ExtenderController(robot);
        ArmController armController = new ArmController(robot);
        CollectScoreController collectScoreController = new CollectScoreController(armController,extenderController,fourbarController,clawPositionController,robot);
        ColorController colorController = new ColorController(robot);
        clawPositionController.update();
        clawRotateController.update();
        clawController.update();
        extenderController.update(extenderTargetPosition);
        fourbarController.update();
        armController.update(armTargetPosition);
        intakeController.update();
        colorController.update();
        collectScoreController.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        //intake1 intake2 fourbar clawposition
            if (currentGamepad1.a == true) {
                clawRotateController.currentStatus= ClawRotateController.ClawRotateStatus.ROTATE;
            } else {
                clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.INIT;
            }
            if (currentGamepad1.b == true) {
                intakeController.intake1.setPower(0.1);
            } else {
                intakeController.intake1.setPower(0);
            }
            if (currentGamepad1.x == true) {
                intakeController.intake2.setPower(0.1);
            } else {
                intakeController.intake2.setPower(0);
            }
            if (currentGamepad1.y == true) {
                fourbarController.fourbar.setPosition(0.1);
            } else {
                fourbarController.fourbar.setPosition(0);
            }

            clawPositionController.update();
            clawRotateController.update();
            clawController.update();
            extenderController.update(extenderTargetPosition);
            fourbarController.update();
            armController.update(armTargetPosition);
            intakeController.update();
            colorController.update();
            collectScoreController.update();
        }
    }
}
