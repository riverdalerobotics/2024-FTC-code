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
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
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


        double max = Math.max(Math.abs(rightFrontSpeed), Math.abs(rightBackSpeed));
        max = Math.max(max,Math.abs(leftFrontSpeed));
        max = Math.max(max, Math.abs(leftBackSpeed));

        if(max > 1) {
            rightBackSpeed /= max;
            rightFrontSpeed /= max;
            leftBackSpeed /= max;
            leftFrontSpeed /= max;
        }

        frontLeft.setPower(leftFrontSpeed/0.5);
        frontRight.setPower(rightFrontSpeed/0.5);
        backRight.setPower(rightBackSpeed/0.5);
        backLeft.setPower(leftBackSpeed/0.5);

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
        double rotX = strafe * Math.cos(-yaw) - fwd* Math.sin(-yaw);
        double rotY = strafe * Math.sin(-yaw) + fwd * Math.cos(-yaw);
        moveMechChassis(rotY, rotX, turn);
    }

    /**
     * This code allows the robot to return to 0 from where ever on the field as long as there is
     * nothing in the way
     *
     * @param xPos the x position relative to 0
     * @param yPos the y position relative to 0
     * @param yaw the yaw of the robot
     * @param kp the proportional constant for the p controller TODO: this might need to be PID
     */
    public void returnToZero(double xPos, double yPos, double yaw, double kp){
        double xSpeed = -xPos*kp;
        double ySpeed = -yPos*kp;
        double turnSpeed = -yaw*kp;
        fieldOriented(yaw, ySpeed, xSpeed, turnSpeed);

    }

    /**
     * This code allows you to go to a desired position from anywhere on the field as long as
     * there is nothing in the way
     *
     * @param xPos the x position relative to 0
     * @param yPos the y position relative to 0
     * @param yaw the yaw of the robot
     * @param kp the proportional constant for the p controller TODO: this might need to be PID
     * @param xResult the desired x position relative to 0 in whatever unit the xPos is
     * @param yResult the desired y position relative to 0 in whatever unit the yPos is
     * @param turnResult the desired angle of the robot relative to 0 in whatever unit the yaw is
     */
    public void goToPosition(double xPos, double yPos, double yaw, double kp, double xResult, double yResult, double turnResult){
        double xError = xPos-xResult;
        double yError = yPos-yResult;
        double turnError = yaw - turnResult;
        double xSpeed = xError*kp;
        double ySpeed = yError*kp;
        double turnSpeed = turnError*kp;
        fieldOriented(yaw, ySpeed, xSpeed, turnSpeed);
    }
}

