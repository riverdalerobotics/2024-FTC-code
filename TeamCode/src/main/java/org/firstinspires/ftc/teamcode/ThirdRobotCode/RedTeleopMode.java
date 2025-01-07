package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Red Pinkie Pie Teleop", group = "Linear OpMode")

public class RedTeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
    MultipleTelemetry telemetryA;


    public void runOpMode() throws InterruptedException{
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        double xPos, yPos, heading;
        char teamColor ='r';
        SparkFunOTOS.Pose2D pose2D;
        CRServo intakeServo;
        Servo wrist;
        RevColorSensorV3 colorSensor;
        IMU imu;
        SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0, 0, 180);
        Command commands;
        SparkFunOTOS otos;
        //imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();
        otos.setOffset(startPos);
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
        OI oi = new OI(gamepad1, gamepad2);

        armPivot = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        leftSlideExtend = hardwareMap.get(DcMotorEx.class, "leftExtends");
        rightSlideExtend = hardwareMap.get(DcMotorEx.class, "rightExtends");
        ArmSubsystem arm = new ArmSubsystem(armPivot, telemetryA);
        SlideSubsystem slides = new SlideSubsystem(rightSlideExtend, leftSlideExtend, telemetryA);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        boolean hasPeice = false;
//        armPivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setDirection(DcMotorEx.Direction.REVERSE);
        armPivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        leftSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10, 0, 0, 0);
        commands = new Command(arm, slides, chassis, intake, pidfCoefficients);
        slides.useRunUsingEncoders();

    while(opModeIsActive()) {
            double speed = Math.pow(gamepad1.left_stick_y, 5/3);
            double strafe = Math.pow(gamepad1.left_stick_x, 5/3);
            double turn = 0.8*Math.pow(gamepad1.right_stick_x, 5/3);
            pose2D = otos.getPosition();
            xPos = pose2D.x*(-3.048);
            yPos = pose2D.y*(-3.048);
            heading = pose2D.h;
            if(heading<0){
                heading+=360;
            }
            chassis.fieldOriented(heading, -speed, strafe, turn);
            if(gamepad1.start){
                otos.setPosition(startPos);
            }

            if(gamepad1.right_bumper){
                chassis.goToPosition(xPos, yPos, heading, -0.03, rotKp, 43, -15, 135);
            }
            if(gamepad2.left_bumper){
                commands.scoreBucket();
            }
            if(gamepad2.right_trigger>=0.3){
                commands.goToZero();
            }
            if(gamepad2.left_trigger>=0.3){
                commands.intake(teamColor, gamepad2, gamepad1);
            }
            if(gamepad2.right_bumper){
                commands.spit();
            }
            if(gamepad2.a){
                slides.moveSlide(-0.5);
            }
            if (gamepad2.start){
                slides.leftSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.leftSlideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.a){
                commands.climb();
            }
            if(gamepad2.left_stick_button){
                armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivot.setPower(gamepad2.right_stick_y);
            }
            if(gamepad2.back){
                armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.addData("Arm pos",arm.getPos());
            telemetry.addData("Arm Target", armPivot.getTargetPosition());
            telemetry.addData("Arm Power", armPivot.getPower());
            telemetry.addData("SlidesPos", slides.getSlidePos());
            telemetry.addData("colour", intake.getColour());
            telemetry.addData("R", colorSensor.red());
            telemetry.addData("G", colorSensor.green());
            telemetry.addData("B", colorSensor.blue());
            telemetry.update();
        }
    }

}
