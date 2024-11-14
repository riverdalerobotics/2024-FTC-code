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
        CRServo intakeServo;
        Servo wrist;
        ColorSensor colorSensor;
        DcMotor rightFront;
        DcMotor rightBack;
        DcMotor leftFront;
        DcMotor leftBack;
        IMU imu;
        SparkFunOTOS otos;
        //imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
//        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        colorSensor = hardwareMap.get(ColorSensor.class, "ColourSensor");
        ChassisSubsystem chassis = new ChassisSubsystem(frontLeft, frontRight, backLeft, backRight, otos);
        //IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist, colorSensor);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        boolean hasPeice = false;
        while(opModeIsActive()) {
            double speed = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            chassis.moveRobotMech(speed, strafe, turn);
//            char colour = intake.getColour();
//            if(colour == 'b'){
//
//                Thread.sleep(500);
//                intake.spinIntake(0);
//                hasPeice = true;
//            }
//            else if(!hasPeice){
//                intake.spinIntake(-0.7);
//            }
//            telemetry.addData("Colour: ", intake.getColour());
//            telemetry.update();
        }
    }

}
