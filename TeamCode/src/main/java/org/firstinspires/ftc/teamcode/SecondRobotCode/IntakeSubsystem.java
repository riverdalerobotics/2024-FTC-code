package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeSubsystem{

    CRServo intake;
    Servo wrist;

    public IntakeSubsystem(CRServo intake, Servo wrist){
        this.intake = intake;
        this.wrist = wrist;



    }
    public void setWristPosition(double pos){
        wrist.setPosition(pos);
    }

    public double  getWristPosition(){
        return wrist.getPosition();
    }

    public void spinTake(double power){
        intake.setPower(power);
    }

    public void wristToZero(){
        wrist.setPosition(0);
      //  wrist.getCurrePosition();

    }
}



