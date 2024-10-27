package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{

    static CRServo intakeMotor;
    static CRServo upDownMotor;

    public static void intakeSpin(double power) throws InterruptedException {


        intakeMotor.setPower(power);
        intakeMotor.wait(1000);
    }
    public static void intakeNonoSpin(double power) {
        intakeMotor.setPower(power);
    }
    public static void intakeSpit(double power) {
        intakeMotor.setPower(power);
    }
    public static void Up(double power) {
         upDownMotor.setPower(power);
    }
    public static void Down(double power) {
        upDownMotor.setPower(power);
    }
    public void hello(){

    }
}
