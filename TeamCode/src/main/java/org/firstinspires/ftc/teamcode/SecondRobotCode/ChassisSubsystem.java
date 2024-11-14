package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ChassisSubsystem {
    double rightBackSpeed;
    double rightFrontSpeed;
    double leftBackSpeed;
    double leftFrontSpeed;

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    //backRight.

    public ChassisSubsystem(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive){

        this.frontLeft = frontLeftDrive;
        this.frontRight = frontRightDrive;
        this.backRight = backRightDrive;
        this.backLeft = backLeftDrive;
        //this.FLspeed  = 0.5;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /**
     * This is kinda source code for mech drive...
     * @param fwd forward speed
     * @param strafe strafe speed
     * @param turn spin speed
     * */
    public void moveRobotMech(double fwd, double strafe, double turn){
        rightBackSpeed = fwd - turn + strafe;
        leftBackSpeed = fwd + turn + strafe;
        rightFrontSpeed = fwd - turn - strafe;
        leftFrontSpeed = fwd + turn - strafe;

        frontLeft.setPower(leftFrontSpeed);
        frontRight.setPower(rightFrontSpeed);
        backRight.setPower(rightBackSpeed);
        backLeft.setPower(leftBackSpeed);
    }
}
