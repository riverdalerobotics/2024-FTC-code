package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{

    static CRServo intake;
    static CRServo lift;
    public IntakeSubsystem(CRServo intake, CRServo lift){
        this.intake = intake;
        this.lift = lift;

    }
    public static void intakeSpin(double power) throws InterruptedException {

        intake.setPower(power);
        intake.wait(1000);
    }

    //What is thsi name?? nonoSpin?
    public static void intakeNonoSpin() {
        intake.setPower(0);
    }
    public static void intakeSpit(double power) {
        intake.setPower(power);
    }
    public static void Up(double power) {
        lift.setPower(power);
    }
    public static void Down(double power) {
        lift.setPower(power);
    }
}
