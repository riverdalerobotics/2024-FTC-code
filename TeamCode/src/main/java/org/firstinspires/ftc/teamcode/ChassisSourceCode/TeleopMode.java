package org.firstinspires.ftc.teamcode.ChassisSourceCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "Field Oriented Test OpMode", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor backLeftDriveMotor;
    private DcMotor backRightDriveMotor;
    IMU imu = hardwareMap.get(IMU.class, "imu");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    ));



    public void runOpMode(){
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRight");
        ChassisSubsystem chassis = new ChassisSubsystem(
                frontLeftDriveMotor,
                frontRightDriveMotor,
                backLeftDriveMotor,
                backRightDriveMotor
        );
        waitForStart();
        imu.initialize(parameters);
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();


        while(opModeIsActive()){
            double yaw = robotOrientation.getYaw() - Constants.ChassisConstants.YAWOFFSET;
            if (yaw<0){
                yaw += 360;
            }
            double fwdSpeed = -gamepad1.left_stick_y; //Inverted
            double strafeSpeed = gamepad1.left_stick_x;
            double turnSpeed = gamepad1.right_stick_x;

            chassis.fieldOriented(yaw, fwdSpeed, strafeSpeed, turnSpeed);
            telemetry.addData("Yaw", yaw);
        }
    }

}
