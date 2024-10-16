package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem {

    CRServo intakeMotor;
    CRServo upDownMotor;

    public void intakeSpin() {
        intakeMotor.setPower(1);
    }
    public void intakeNonoSpin() {
        intakeMotor.setPower(0);
    }
    public void intakeSpit() {
        intakeMotor.setPower(-1);
    }
    public void Up() {
         upDownMotor.setPower(1);
    }
    public void Down() {
        upDownMotor.setPower(-1);
    }

    }
}
