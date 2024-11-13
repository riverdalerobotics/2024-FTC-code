package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp (name = "Yes Teleop", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Mech drive
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;


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





    }

}
