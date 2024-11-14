package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//impo
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp (name = "Sana's Teleop", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Mech drive
    private DcMotorSimple frontLeftDrive;
    private DcMotorSimple backLeftDrive;
    private DcMotorSimple frontRightDrive;
    private DcMotorSimple backRightDrive;


    // what is this for??
    private DcMotor armMotor;

    // arm extender ??
    private DcMotor armExtend;

    // intake!!

    private CRServo liftMotor;
    private CRServo intakeMotor;


    public WebcamName camera;
    //April

    ChassisSubsystem chassis;
    ArmSubsytem arm;
    armExtender armExtender;
    IntakeSubsystem intake;


    public void runOpMode(){
        //
        //frontLeftDrive = hardwareMap();

    chassis = new ChassisSubsystem(frontLeftDrive, frontRightDrive,backLeftDrive,backRightDrive);
    arm = new ArmSubsytem(armMotor);
    armExtender = new armExtender(armExtend);
    intake = new IntakeSubsystem(intakeMotor, liftMotor);

        frontLeftDrive = hardwareMap.get(DcMotorSimple.class ,"motorLeftF");
        frontRightDrive = hardwareMap.get(DcMotorSimple.class, "motorRightF");
        backRightDrive = hardwareMap.get(DcMotorSimple.class, "motorRightB");
        backLeftDrive = hardwareMap.get(DcMotorSimple.class, "motorLeftB");



        waitForStart();

        double driveYaxis;
        double driveXaxis;
        double rotatePwr;
        double armSpeed;
        
        while (opModeIsActive()){
            driveYaxis= gamepad1.left_stick_y;
            driveXaxis = gamepad1.left_stick_x;
            rotatePwr = gamepad1.right_stick_x;
            chassis.moveRobotMech(driveYaxis, driveXaxis, rotatePwr);


            telemetry.addData("Status", "wobot is on :3");
            telemetry.update();



        }









    }

}
