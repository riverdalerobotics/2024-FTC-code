package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{


    //TODO: Intkae shoud use setPower and Lift should use set Pos
    //TODO: intkae is continous ,
    CRServo intake;
    Servo lift;


    public IntakeSubsystem(CRServo intake, Servo lift){
        this.intake = intake;
        this.lift = lift;

    }
    public void Up(double pos) {
        lift.setPosition(pos);
    }
    public void Down(double pos) {
        lift.setPosition(-pos);
    }
//    public void spinTake(double power){
//        intake.setPower(power);
//    }
}
