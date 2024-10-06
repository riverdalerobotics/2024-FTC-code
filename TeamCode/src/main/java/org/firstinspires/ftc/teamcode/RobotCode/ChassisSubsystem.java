package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ChassisSubsystem {
    DcMotor leftDrive;
    DcMotor rightDrive;


    public ChassisSubsystem(DcMotor left, DcMotor right){
        this.leftDrive = left;
        this.rightDrive = right;
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void drive(double speed, double turn){
        double leftSpeed = speed-turn;
        double rightSpeed = speed+turn;
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

}
