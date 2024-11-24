package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{

    CRServo intake;
    Servo wrist;

    public IntakeSubsystem(CRServo intake, Servo lift){
        this.intake = intake;
        this.wrist = lift;

    }
    public void Up(double pos) {
        wrist.setPosition(pos);
    }
    public void Down(double pos) {
        wrist.setPosition(-pos);
    }
    public void spinTake(double power){
        intake.setPower(power);
    }
    public void resetPos(){
        wrist.setPosition(0);
    }
}
