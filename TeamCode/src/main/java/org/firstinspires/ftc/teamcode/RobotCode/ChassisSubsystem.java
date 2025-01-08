package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class ChassisSubsystem {
    DcMotor leftDrive;
    DcMotor rightDrive;
    IMU imu;
    IMU.Parameters myIMUparameters;
    YawPitchRollAngles robotOrientation;

    public ChassisSubsystem(DcMotor left, DcMotor right, IMU imu){
        this.leftDrive = left;
        this.rightDrive = right;
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

    }

    public void drive(double speed, double turn){
        double leftSpeed = speed-turn;
        double rightSpeed = speed+turn;
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    public void moveRobotTest(double speed){
        leftDrive.setPower(0.1);
        rightDrive.setPower(0.1);
    }


    public double getYaw(){
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }



}
