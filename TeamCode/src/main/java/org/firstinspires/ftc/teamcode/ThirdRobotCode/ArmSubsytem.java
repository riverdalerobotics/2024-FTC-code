package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* cos(pivot angle) = limit/arm
*
*/
public class ArmSubsytem {

    DcMotor armPivot;
    DcMotor rightExtend;
    DcMotor leftExtend;

    public ArmSubsytem(DcMotor armPivot, DcMotor rightExtend, DcMotor leftExtend){
    this.armPivot = armPivot;
    this.leftExtend = leftExtend;
    this.rightExtend = rightExtend;

    this.leftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
    this.rightExtend.setDirection(DcMotorSimple.Direction.REVERSE); // may reverse these

    // set initial position for motors
    }

    public void extendArm(double armExtendInput){

    }
    public void pivotArm(double armPivotInput, double power){// what is power for?

    }
}

