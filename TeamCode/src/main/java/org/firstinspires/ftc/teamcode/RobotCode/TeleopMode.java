package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCode.ArmSubsystem;

@TeleOp (name = "Nicolas Teleop", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode(){
        waitForStart();
        DcMotor left;
        DcMotor right;
        DcMotor armPivot;
        CRServo intakeServo;
        Servo wrist;
        left = hardwareMap.get(DcMotor.class, "LeftMotor");
        right = hardwareMap.get(DcMotor.class, "RightMotor");
        armPivot = hardwareMap.get(DcMotor.class, "ArmPivot");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ChassisSubsystem chassis = new ChassisSubsystem(left, right);
        ArmSubsystem arm = new ArmSubsystem(armPivot);
        Commands commands = new Commands(armPivot, wrist, intakeServo);
        OI oi = new OI(gamepad1, gamepad2);
       // IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist);
        while(opModeIsActive()){
           // IntakeSubsystem.intakePowerOn(0);
//
//            if(OI.testArmAngle()){
//               commands.moveArm90Degrees();
//
//            }
//
//            double power = gamepad1.left_stick_y;
//            armPivot.setPower(power*0.3);
            telemetry.addData("Motor Position", armPivot.getCurrentPosition()/(537.6*360*5));
            telemetry.update();


        }

    }

}
