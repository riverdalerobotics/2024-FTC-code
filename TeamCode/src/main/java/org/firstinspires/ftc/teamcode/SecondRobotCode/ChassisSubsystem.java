package org.firstinspires.ftc.teamcode.SecondRobotCode;

import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.WHEEL_BASE;


import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.kA;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.kStatic;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.kV;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ChassisConstants.RUN_USING_ENCODER;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ChassisSubsystem extends MecanumDrive{
    double rightBackSpeed;
    double rightFrontSpeed;
    double leftBackSpeed;
    double leftFrontSpeed;

    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;

    public double tempSpeed, tempStrafe, tempTurn;
    public double currentSpeed, currentStrafe, currentTurn;

    IMU imu;
    private List<DcMotorEx> motors;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.0;

    public double VX_WEIGHT = 1;
    public double VY_WEIGHT = 1;
    public double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;


    public ChassisSubsystem(DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive, DcMotorEx backLeftDrive, DcMotorEx backRightDrive, IMU imu){
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);



        this.frontLeft = frontLeftDrive;
        this.frontRight = frontRightDrive;
        this.backRight = backRightDrive;
        this.backLeft = backLeftDrive;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

       this.imu = imu;




        motors = Arrays.asList(frontLeft, frontRight, backRight, backLeft);
        if (RUN_USING_ENCODER) {
            setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotorEx.setDirection()

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
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

    ///////////////////////////////////////////////////////////////////////
    /**
     * Mecanum Logic Slewed (test after everything is done) //TODO: Test this and change for moveMechChassis()
     * @param speed forward speed
     * @param strafe strafe speed
     * @param turn spin speed
     * */
    public void moveMechChassiSlew(double speed, double strafe, double turn){

        currentSpeed = slew(tempSpeed, speed, 0.03);
        currentStrafe = slew(tempStrafe, strafe, 0.03);
        currentTurn = slew(tempTurn, turn, 0.5); //helped by Jason Wu

        tempSpeed=currentSpeed;
        tempStrafe=currentStrafe;
        tempTurn=currentTurn; //basically the previous output variable is obtained from adding 0.05 (the slew rate) to the previous turn speed and this is updated 50 times a second (at least for frc it's something like that)

        rightBackSpeed = tempSpeed + tempTurn - tempStrafe;
        leftBackSpeed = tempSpeed - tempTurn + tempStrafe;
        rightFrontSpeed = tempSpeed + tempTurn + tempStrafe;
        leftFrontSpeed = tempSpeed - tempTurn - tempStrafe;

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


    //Taken from https://www.reddit.com/r/FTC/comments/3vx37h/motor_acceleration/
   public double slew (double prev, double input, double slewRate) {
    if (slewRate < (Math.abs(prev - input))) {
      if (prev - input < 0) {
        return ( prev + slewRate);
      } else return (prev - slewRate);
    }
    else return input;
  }

    /////////////////////////////////////////////////////////////////////////

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


    //pathplanner stuff

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(DriveConstants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(DriveConstants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}


