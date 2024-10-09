package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    CRServo intakeServo;

public void IntakePowerOn() {
    intakeServo.setPower(0.5);
}

public void intakePowerOff(){
intakeServo.setPower(0);

}
}


