package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ChassisSubsystem {
    double rightBackSpeed;
    double rightFrontSpeed;
    double leftBackSpeed;
    double leftFrontSpeed;

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    IMU gyro;


    public ChassisSubsystem(IMU gyro, DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive){

        this.frontLeft = frontLeftDrive;
        this.frontRight = frontRightDrive;
        this.backRight = backRightDrive;
        this.backLeft = backLeftDrive;
        this.gyro = gyro;
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        gyro.initialize(parameters);

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

    public static double pitch(IMU gyro){
        YawPitchRollAngles yawPitchRollAngles = gyro.getRobotYawPitchRollAngles();
        return yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
    }

}
