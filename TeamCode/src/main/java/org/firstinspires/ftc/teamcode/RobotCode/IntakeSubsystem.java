package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    static CRServo intakeServo;
    static Servo wrist;

public static void intakePowerOn(double power) {
    intakeServo.setPower(power);
}

public static void intakePowerOff(){
intakeServo.setPower(0);

}

public static void moveWrist(double position){

    wrist.setPosition(position);
}
}


