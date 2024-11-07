package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.transition.Slide;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TestOpMode", group = "Linar OpMode")

public class TeleopMode {
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

    public void runOpMode(){
        gyro = hardwareMap.get(IMU.class, "Gyro");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        otos = hardwareMap.get(SparkFunOTOS.class, "OTOS");
        armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("ArmEncoder", armPivot.getCurrentPosition());

        chassis = new ChassisSubsystem(gyro, frontLeft, frontRight, backLeft, backRight, otos);
        arm = new ArmSubsystem(armPivot);


        double fwdPwr;
        double strafePwr;
        double rotationPwr;
        double armPwr;

        while(linearOpMode.opModeIsActive()) {
            fwdPwr = -gamepad1.left_stick_y;
            strafePwr = -gamepad1.left_stick_x;
            rotationPwr = -gamepad1.right_stick_x;
            armPwr = -gamepad2.left_stick_y;
        }
    }

}
