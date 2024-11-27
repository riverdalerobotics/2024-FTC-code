package org.firstinspires.ftc.teamcode.RookieBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Rookie Bot TeleOP")
public class RookieTeleOp extends LinearOpMode {

        DcMotor leftDrive;
        DcMotor rightDrive  ;
        DcMotor arm;
        DcMotor wrist;
        Servo claw;
        CRServo intake;
        ChassisSubsystem chassis;
        ArmSubsystem armSub;
        IntakeSubsystem intakeSub;
        DcMotor intakeMotor;

    public void runOpMode() throws InterruptedException {

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        armSub = new ArmSubsystem(arm);
        intakeSub = new IntakeSubsystem(intake, claw, intakeMotor);
        waitForStart();

        double speed;
        double turn;

        while(opModeIsActive()){

            speed = -gamepad1.left_stick_y*0.3;
            turn = gamepad1.right_stick_x*0.3;

            chassis.drive(speed,turn);
            telemetry.addData("Arm Position", armSub.getPosInDegrees());
            telemetry.addData("Y axis Pwr", speed);
            telemetry.addData("X axis Pwr", turn);
            telemetry.update();
        }
    }
}
