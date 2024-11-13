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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

@TeleOp (name = "TestOpMode", group = "Linear OpMode")

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
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        wrist = hardwareMap.get(Servo.class, "wrist");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColourSensor");
        IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist, colorSensor);
        waitForStart();

        while(opModeIsActive()) {
            char colour = intake.getColour();
            if(colour == 'b'){
                while(colour == 'b'){

                }
                intake.spinIntake(0.1);
                Thread.sleep(100);
                intake.spinIntake(0);
            }
            else{
                intake.spinIntake(-0.5);
            }
            telemetry.addData("Colour: ", intake.getColour());
            telemetry.update();
        }
    }

}
