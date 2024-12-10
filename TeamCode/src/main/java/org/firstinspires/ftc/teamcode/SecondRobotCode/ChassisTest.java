package org.firstinspires.ftc.teamcode.SecondRobotCode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ChassisTest {

    double rightBackSpeed;
    double rightFrontSpeed;
    double leftBackSpeed;
    double leftFrontSpeed;

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    IMU gyro;


    public ChassisTest(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive, IMU gyro){

        this.frontLeft = frontLeftDrive;
        this.frontRight = frontRightDrive;
        this.backRight = backRightDrive;
        this.backLeft = backLeftDrive;
        this.gyro = gyro;

        this.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.backLeft.setDirection(DcMotor.Direction.REVERSE);

//        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//        ));
//        gyro.initialize(parameters);

    }
    /**
     * This is kinda source code for mech drive...
     * @param fwd forward speed
     * @param strafe strafe speed
     * @param turn spin speed
     * */

    public void moveRobotMech(double fwd, double strafe, double turn){

        rightBackSpeed = fwd - turn + strafe;
        leftBackSpeed = fwd + turn - strafe;
        rightFrontSpeed = fwd - turn - strafe;
        leftFrontSpeed = fwd + turn + strafe;


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
    /**
     * <p>
     * Field oriented allows the robot to drive relative to a starting position when given the yaw
     * of the robot. The math behind this can be found here https://matthew-brett.github.io/teaching/rotation_2d.html
     * you can find the source code here: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
     *<p/>
     * <b><u>ALL MOTORS CAP AT ONE</u></b>
     * <p/>
     * @param yaw the yaw of the robot, <u\>must be updated everytime the opmode loop is run
     * @param fwd the foreword speed of the robot relative to the field
     * @param strafe the strafe speed of the robot relative to the field
     * @param turn the spin speed of the robot
     * */
    public void fieldOriented(double yaw, double fwd, double strafe, double turn){
        double rotX = strafe * Math.cos(yaw) - fwd* Math.sin(yaw);
        double rotY = strafe * Math.sin(yaw) + fwd * Math.cos(yaw);
        moveRobotMech(rotY, rotX, turn);
    }





}
