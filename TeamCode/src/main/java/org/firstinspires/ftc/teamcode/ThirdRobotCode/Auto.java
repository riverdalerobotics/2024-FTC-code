package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import android.app.ApplicationErrorReport;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pinkie Pie Auto 3 piece", group = "Linear OpMode")
public class Auto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        IMU gyro;
        DcMotorEx rightSlideExtend;
        DcMotorEx leftSlideExtend;
        Servo leftIntake;
        Servo rightIntake;
        DcMotorEx armPivot;
        double xPos, yPos, heading;
        char teamColor ='b';
        SparkFunOTOS.Pose2D pose2D;
        CRServo intakeServo;
        Servo wrist;
        RevColorSensorV3 colorSensor;
        IMU imu;
        SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0, 0, 180);
        MultipleTelemetry telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Command commands;
        SparkFunOTOS otos;
        //imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();
        otos.setOffset(startPos);
        otos.setPosition(startPos);
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        double rotKp = 0.5;
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColourSensor");
        ChassisSubsystem chassis = new ChassisSubsystem(frontLeft, frontRight, backLeft, backRight, otos);
        IntakeSubsystem intake = new IntakeSubsystem(intakeServo, leftIntake, colorSensor, rightIntake, telemetry);

        ElapsedTime stopwatch = new ElapsedTime();
        armPivot = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        leftSlideExtend = hardwareMap.get(DcMotorEx.class, "leftExtends");
        rightSlideExtend = hardwareMap.get(DcMotorEx.class, "rightExtends");
        ArmSubsystem arm = new ArmSubsystem(armPivot, telemetryA);
        SlideSubsystem slides = new SlideSubsystem(rightSlideExtend, leftSlideExtend, telemetryA);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        armPivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setDirection(DcMotorEx.Direction.REVERSE);
        armPivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10, 0, 0, 0);
        commands = new Command(arm, slides, chassis, intake, pidfCoefficients);
        slides.useRunUsingEncoders();
        waitForStart();
        pose2D = otos.getPosition();
        xPos = pose2D.x*(-3.048);
        yPos = pose2D.y*(-3.048);
        heading = pose2D.h;
        if(heading<0){
            heading+=360;
        }
        while(chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 46, -17, 130)){
            //chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 45, -20, 135);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.update();
        }
        commands.scoreBucket();
        while(slides.leftSlideExtend.isBusy()){
        }
        commands.spit();
        commands.goToZero();
        while(chassis.goToPosition(xPos, yPos, heading, -0.04, rotKp, 60, -40, 180)){
            //chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 45, -20, 135);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.update();
        }
        chassis.fieldOriented(heading, 0.05, 0, 0);


        stopwatch.reset();
        while (stopwatch.time()<4){
            commands.intake(teamColor, gamepad2, gamepad1);
        }

        commands.goToZero();
        while(chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 49, -17, 130)){
            //chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 45, -20, 135);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.update();
        }
        commands.scoreBucket();
        while(slides.leftSlideExtend.isBusy()){
        }
        commands.spit();
        commands.goToZero();
        while(chassis.goToPosition(xPos, yPos, heading, -0.06, rotKp, 26, -34, 180)){
            //chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 45, -20, 135);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.update();
        }
        chassis.fieldOriented(heading, 0.05, 0, 0);
        commands.intake(teamColor, gamepad2, gamepad1);
        commands.goToZero();
        while(chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 49, -17, 130)){
            //chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 45, -20, 135);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.addData("Stopwatch time:",stopwatch.time());
            telemetry.update();

        }
        commands.scoreBucket();
        while(slides.leftSlideExtend.isBusy()){}
        commands.spit();
        commands.goToZero();
        while(arm.armPivotMotor.isBusy()){}
        arm.pivotArm(-1, 0.1, pidfCoefficients);
        slides.goToPosWithSpeed(-0.1, 0.1);
    }
}
