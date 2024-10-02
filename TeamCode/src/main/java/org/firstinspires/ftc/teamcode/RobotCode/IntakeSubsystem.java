package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {

    DcMotor intakeMotor;

public void IntakePowerOn(){

    intakeMotor.setPower(1);
    }

public void intakePowerOff(){

    intakeMotor.setPower(0);
}
}


