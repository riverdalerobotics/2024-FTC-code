package org.firstinspires.ftc.teamcode.ThirdRobotCode;

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.ArmGoToScore;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.GoToZero;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.MoveSidesManually;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.MoveSlides;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.ScoreBucket;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.SlideDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.IntakeDefaultCommand;

@Config
@TeleOp (name = "TestCommandOpMode", group = "LinerOpMode")
public class TestTeleop extends CommandOpMode {
    SlideSubsystem slides;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    DcMotorEx armMotor;
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    CRServo intakeServo;
    Servo leftWrist;
    Servo rightWrist;
    ColorRangeSensor colour;
    GamepadEx operator;
    SlideDefaultCommand slideDefaultCommand;
    ArmDefaultCommand armDefaultCommand;
    IntakeDefaultCommand intakeDefaultCommand;
    MultipleTelemetry telemetryA;
    FtcDashboard dashboard;
    public static double speedLimit = 0.1;
    public static PIDFCoefficients slidesPidf = Constants.SlideConstants.slidesPID;
    RevColorSensorV3 sensor;
// set the clock speed on this I2C bus to 400kHz:


    @Override
    public void initialize(){
        sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        dashboard = FtcDashboard.getInstance();
        operator = new GamepadEx(gamepad2);
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftExtends");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightExtends");
        intakeServo = hardwareMap.get(CRServo.class, "intake");
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        colour = hardwareMap.get(ColorRangeSensor.class, "ColourSensor");
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new IntakeSubsystem(intakeServo, leftWrist, sensor, rightWrist, telemetry);
        slides =  new SlideSubsystem(rightSlide, leftSlide, telemetryA);
        arm = new ArmSubsystem(armMotor, telemetryA);
        slideDefaultCommand = new SlideDefaultCommand(slides);
        armDefaultCommand = new ArmDefaultCommand(arm, telemetry);
        intakeDefaultCommand = new IntakeDefaultCommand(intake, telemetry);
        register(arm, intake, slides);
        schedule(armDefaultCommand, intakeDefaultCommand, slideDefaultCommand);
        slides.setDefaultCommand(slideDefaultCommand);
        arm.setDefaultCommand(armDefaultCommand);
        intake.setDefaultCommand(intakeDefaultCommand);


    }
    @Override
    public void run(){
        super.run();
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
        Button test = new GamepadButton(
                operator, GamepadKeys.Button.LEFT_STICK_BUTTON
        ).whileHeld(
                new MoveSidesManually(slides, telemetryA, speedLimit)
        );
        telemetryA.addData("slidesPower", leftSlide.getPower());
        telemetryA.update();
    }


}
