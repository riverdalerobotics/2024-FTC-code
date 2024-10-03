package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    DcMotor intakeMotor;
    public IntakeSubsystem(DcMotor intakeMotor){
        this.intakeMotor = intakeMotor;
    }

    public void intake(double power){
        this.intakeMotor.setPower(power);
    }
}
