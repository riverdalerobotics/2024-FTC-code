package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.transition.Slide;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

@TeleOp (name = "Teleopsmodes", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU gyro;
    DcMotor rightSlideExtend;
    DcMotor leftSlideExtend;

    DcMotorEx armPivot;

    ChassisSubsystem chassis;
    ArmSubsystem arm;
    SlideSubsystem slides;
    SparkFunOTOS otos;

    public void runOpMode() throws InterruptedException{
        double xPos, yPos, heading;
        SparkFunOTOS.Pose2D pose2D;
        CRServo intakeServo;
        Servo wrist;
        ColorSensor colorSensor;
        IMU imu;
        SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0, 0, 180);

        SparkFunOTOS otos;
        //imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();
        otos.setOffset(startPos);
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        colorSensor = hardwareMap.get(ColorSensor.class, "ColourSensor");
        ChassisSubsystem chassis = new ChassisSubsystem(frontLeft, frontRight, backLeft, backRight, otos);
//        IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist, colorSensor);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        double rotKp = 0.35;
        boolean hasPeice = false;
        while(opModeIsActive()) {
            double speed = Math.pow(gamepad1.left_stick_y, 5/3);
            double strafe = Math.pow(gamepad1.left_stick_x, 5/3);
            double turn = Math.pow(gamepad1.right_stick_x, 5/3);

            //chassis.moveRobotMech(speed, strafe, turn);

//            char colour = intake.getColour();
//            if(colour == 'b'&&!hasPeice){
//                intake.spinIntake(0);
//                hasPeice = true;
//            }
//            else if(!hasPeice){
//                intake.spinIntake(-1);
//            }
//            telemetry.addData("Colour: ", intake.getColour());

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

            if(gamepad1.a){
                chassis.goToPosition(xPos, yPos, heading, 0.03, rotKp, 0, -100, 0);
                //chassis.goToPosition(0, yPos, heading, 0.1, 0,0, 180);
            }

            telemetry.addData("x position:", xPos);
            telemetry.addData("y position:", yPos);
            telemetry.addData("heading position:", heading);
            telemetry.addData("kp:", rotKp);
            telemetry.update();
        }
    }

}
