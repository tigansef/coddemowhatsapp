package org.firstinspires.ftc.teamcode.drive.writtenCode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class RobotMap {
    public DcMotorEx turret;
    public DcMotorEx arm;
    public DcMotorEx extender;
    public CRServo intake2;
    public CRServo intake1;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo clawRotate;
    public Servo clawPosition;
    public Servo fourbar;
    public MotorConfigurationType mctIntake, mctExtenderLeft ,mctExtenderRight;
    public ColorSensor colorSensor;
    public RobotMap(HardwareMap Init)
    {
        turret = Init.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = Init.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender = Init.get(DcMotorEx.class, "extender");
        extender.setDirection(DcMotor.Direction.REVERSE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intake1=Init.get(CRServo.class, "intake1");
        intake2=Init.get(CRServo.class, "intake2");
        leftClaw=Init.get(Servo.class, "leftClaw");
        rightClaw=Init.get(Servo.class, "rightClaw");
        clawRotate=Init.get(Servo.class, "clawRotate");
        clawPosition=Init.get(Servo.class, "clawPosition");
        fourbar=Init.get(Servo.class, "fourbar");
        colorSensor = Init.get(ColorSensor.class, "culoare");

    }
}
