package org.firstinspires.ftc.teamcode.ThirdRobotCode.CompCode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.AutoScore;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.ScoreBucket;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.Spit;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ChassisDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.SlideDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.IntakeDefaultCommand;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.OI;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "PinkiePieIsAllAlone:(", group = "Linear OpMode")
public class PinkiePieOnePiece extends CommandOpMode{
    SlideSubsystem slides;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    DcMotorEx armMotor;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    CRServo intakeServo;
    Servo leftWrist;
    Servo rightWrist;
    RevColorSensorV3 colour;
    GamepadEx operator;
    GamepadEx driver;
    ChassisDefaultCommand chassisDefaultCommand;
    ChassisSubsystem chassis;
    SlideDefaultCommand slideDefaultCommand;
    ArmDefaultCommand armDefaultCommand;
    IntakeDefaultCommand intakeDefaultCommand;
    MultipleTelemetry telemetryA;
    FtcDashboard dashboard;
    public static double speedLimit = 0.1;
    public static PIDFCoefficients slidesPidf = Constants.SlideConstants.slidesPID;
    RevColorSensorV3 sensor;
    SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0, 0, 180);
    double heading;
    OI oi;
    Gamepad gamepad;
    SparkFunOTOS.Pose2D pose2D;
    double xPos,yPos;
    SparkFunOTOS otos;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    PathChain pathChain;
    Follower follower;
    Pose startPose = new Pose(9.746, 110.602, 0);

    @Override
    public void initialize() {
        otos = hardwareMap.get(SparkFunOTOS.class, "I2C0");
        otos.calibrateImu();
        otos.resetTracking();
        otos.setOffset(startPos);
        sensor = hardwareMap.get(RevColorSensorV3 .class, "Color");
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        dashboard = FtcDashboard.getInstance();
        operator = new GamepadEx(gamepad2);
        driver = new GamepadEx(gamepad1);
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftExtends");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightExtends");
        intakeServo = hardwareMap.get(CRServo.class, "intake");
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        colour = hardwareMap.get(RevColorSensorV3.class, "ColourSensor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis = new ChassisSubsystem(frontLeft, frontRight, backLeft, backRight, otos);
        intake = new IntakeSubsystem(intakeServo, leftWrist, sensor, rightWrist, telemetry);
        slides =  new SlideSubsystem(rightSlide, leftSlide, telemetryA);
        arm = new ArmSubsystem(armMotor, telemetryA);
        slideDefaultCommand = new SlideDefaultCommand(slides);
        armDefaultCommand = new ArmDefaultCommand(arm, telemetry);
        intakeDefaultCommand = new IntakeDefaultCommand(intake, telemetry);
        chassisDefaultCommand = new ChassisDefaultCommand(chassis, gamepad1, otos);
        register(arm, intake, slides, chassis);
        schedule(armDefaultCommand, intakeDefaultCommand, slideDefaultCommand, chassisDefaultCommand);
        slides.setDefaultCommand(slideDefaultCommand);
        arm.setDefaultCommand(armDefaultCommand);
        intake.setDefaultCommand(intakeDefaultCommand);
        PathChain pathChain;
        Pose startPose = new Pose(9.746, 107.602, Math.PI);
    }

    @Override
    public void run(){
            super.run();
            follower.update();
            if(!follower.isBusy()){
                new AutoScore(slides, arm, intake, telemetryA);

        }
    }
}
