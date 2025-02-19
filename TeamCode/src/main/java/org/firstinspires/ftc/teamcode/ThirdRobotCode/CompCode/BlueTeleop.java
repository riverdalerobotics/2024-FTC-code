package org.firstinspires.ftc.teamcode.ThirdRobotCode.CompCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.Climb;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.ClimbPartTwo;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.GoToZero;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.Intake;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.IntakeStepTwoButFancy;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.ScoreBucket;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.Spit;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ChassisDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.SlideDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.OI;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

@Config
@TeleOp (name = "PinkiePieSad:(", group = "LinerOpMode")
public class BlueTeleop extends CommandOpMode {
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
    char teamColour = 'b';

// set the clock speed on this I2C bus to 400kHz:


    @Override
    public void initialize(){
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;

        otos = hardwareMap.get(SparkFunOTOS.class, "I2C0");
        otos.calibrateImu();
        otos.resetTracking();
        otos.setOffset(startPos);
        sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
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
        chassis.setDefaultCommand(chassisDefaultCommand);




    }
    @Override
    public void run(){
        super.run();
        pose2D = otos.getPosition();
        Button zeroButton = new GamepadButton(
                operator, GamepadKeys.Button.A
        ).whenPressed(
                new GoToZero(slides, arm, intake, telemetryA, Constants.ArmConstants.armPID, slidesPidf)
        );
        Button scoreButton = new GamepadButton(
                operator, GamepadKeys.Button.B
        ).whenPressed(
                new ScoreBucket(slides, arm, intake, telemetryA, Constants.ArmConstants.armPID, slidesPidf)
        );
        Button intakeButton = new GamepadButton(
                operator, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(
                new Intake(intake, slides, arm, teamColour, telemetryA)
        );
        Button climb = new GamepadButton(
                driver, GamepadKeys.Button.A
        ).whenPressed(
                new Climb(arm, slides, intake, telemetryA, Constants.ArmConstants.armPID)
        );
        Button secondClimb = new GamepadButton(
                driver, GamepadKeys.Button.B
        ).whenPressed(
                new ClimbPartTwo(slides, arm, intake, telemetryA, Constants.ArmConstants.armPID, Constants.SlideConstants.slidesPID)
        );

        Button spitButton = new GamepadButton(
                operator, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                new Spit(intake, 'b')
        );
        Button intakeDown = new GamepadButton(
                operator, GamepadKeys.Button.X
        ).whenPressed(
                new MoveIntake(intake, Constants.IntakeConstants.INTAKE_POSITION)
        );
        Button intakeButFancyButton = new GamepadButton(
                operator, GamepadKeys.Button.Y
        ).whenPressed(
                new IntakeStepTwoButFancy(intake, slides, teamColour)
        );

        telemetryA.addData("heading", pose2D.h);
        telemetryA.update();
    }


}
