package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants.ArmSubsystem.CLAW_SERVO_START_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/* cos(pivot angle) = limit/arm
*
*/
public class ArmSubsystem{
    DcMotor armPivotMotor;

    int armPivotMotorPosition;

    public ArmSubsystem(DcMotor armPivot, Servo clawRight, Servo clawLeft){
        this.armPivotMotor = armPivot;
    }

    public void pivotClaw(double clawPivotInput, double power){

    }
    public void pivotArm(double armPivotInput, double power){
        armPivotMotorPosition += (int) armPivotInput;
        armPivotMotor.setPower(power);
        armPivotMotor.setTargetPosition(armPivotMotorPosition);
    }

}
