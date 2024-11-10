package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    CRServo intakeServo;
    Servo wrist;
    public IntakeSubsystem(CRServo intakeServo, Servo wrist){
        this.intakeServo = intakeServo;
        this.wrist = wrist;
    }
public void intakePowerOn(double power) {
    intakeServo.setPower(power);
}

public void intakePowerOff(){
intakeServo.setPower(0);

}

public void moveWrist(double position){

    wrist.setPosition(position);
}
}


