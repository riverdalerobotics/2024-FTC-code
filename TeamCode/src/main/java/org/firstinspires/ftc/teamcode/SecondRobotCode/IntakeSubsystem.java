package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{

    CRServo intakeMotor;

    CRServo upDownMotor;
    public IntakeSubsystem(CRServo intakeMotor, CRServo upDownMotor){
        this.intakeMotor = intakeMotor;
        this.upDownMotor = upDownMotor;
    }

    public void intakeSpin(double power) throws InterruptedException {


        intakeMotor.setPower(power);
        intakeMotor.wait(1000);
    }
    public void intakeNonoSpin(double power) {
        intakeMotor.setPower(power);
    }
    public void intakeSpit(double power) {
        intakeMotor.setPower(power);
    }
    public void Up(double power) {
         upDownMotor.setPower(power);
    }
    public void Down(double power) {
        upDownMotor.setPower(power);
    }
    public void hello(){

    }
}
