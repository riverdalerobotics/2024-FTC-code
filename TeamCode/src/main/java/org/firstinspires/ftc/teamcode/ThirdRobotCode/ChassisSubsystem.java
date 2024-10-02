package org.firstinspires.ftc.teamcode.ThirdRobotCode;

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

    public ChassisSubsystem(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive){

        this.frontLeft = frontLeftDrive;
        this.frontRight = frontRightDrive;
        this.backRight = backRightDrive;
        this.backLeft = backLeftDrive;

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

        //nerf the speed if over absolute 1
        double max = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightBackSpeed));
        max = Math.max(max,Math.abs(leftFrontSpeed));
        max = Math.max(max, Math.abs(leftBackSpeed));

        if(max > 1.0) {
            rightBackSpeed /= max;
            rightFrontSpeed /= max;
            leftBackSpeed /= max;
            leftFrontSpeed /= max;
        }

        frontLeft.setPower(leftFrontSpeed);
        frontRight.setPower(rightFrontSpeed);
        backRight.setPower(rightBackSpeed);
        backLeft.setPower(leftBackSpeed);


    }
}
