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
    public double degToRotation(double deg){
           return deg/360*1425*5;
    }
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode(){
        waitForStart();
        DcMotor left;
        DcMotor right;
        DcMotor armPivot;
        Servo wrist;
        CRServo intakeServo;
        left = hardwareMap.get(DcMotor.class, "LeftMotor");
        right = hardwareMap.get(DcMotor.class, "RightMotor");
        armPivot = hardwareMap.get(DcMotor.class, "ArmPivot");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ChassisSubsystem chassis = new ChassisSubsystem(left, right);
        ArmSubsystem arm = new ArmSubsystem(armPivot);
        Commands commands = new Commands(armPivot, wrist, intakeServo);
        OI oi = new OI(gamepad1, gamepad2);
        double armAngle = armPivot.getCurrentPosition()*360/(1425.2*5);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean moveServo = false;


       // IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist);
        while(opModeIsActive()){
           // IntakeSubsystem.intakePowerOn(0);
//
//
//
//            }

            double speed = gamepad2.left_stick_y;
            double turn = gamepad2.right_stick_x;

            chassis.drive(speed,turn);

            if (gamepad2.b){
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int)degToRotation(90.0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.a){
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int)degToRotation(250.0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.x){
                intakeServo.setPower(0.8);
            }
            else{
                intakeServo.setPower(0);

            }

            if (gamepad2.y){
                intakeServo.setPower(-0.8);
            }
            else{
                intakeServo.setPower(0);
            }


            }
//            double power = gamepad1.left_stick_y;
//            armPivot.setPower(power*0.3);
            telemetry.addData("Motor Position", armPivot.getCurrentPosition()*360/(1425.2*5));
            telemetry.update();


        }

    }


