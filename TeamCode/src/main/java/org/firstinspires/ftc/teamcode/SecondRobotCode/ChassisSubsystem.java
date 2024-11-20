package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);
    }


    /**
     * This is kinda source code for mech drive...
     * @param speed forward speed
     * @param strafe strafe speed
     * @param turn spin speed
     * */
    public void moveMechChassis(double speed, double strafe, double turn){
        rightBackSpeed = speed + turn - strafe;
        leftBackSpeed = speed - turn + strafe;
        rightFrontSpeed = speed + turn + strafe;
        leftFrontSpeed = speed - turn - strafe;


        frontLeft.setPower(leftFrontSpeed);
        frontRight.setPower(rightFrontSpeed);
        backRight.setPower(rightBackSpeed);
        backLeft.setPower(leftBackSpeed);
    }
}
