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
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        armSub = new ArmSubsystem(arm);
        intakeSub = new IntakeSubsystem(intake, claw, intakeMotor);
        chassis = new ChassisSubsystem(leftDrive, rightDrive);
        waitForStart();

        double speed;
        double turn;

        while(opModeIsActive()){

            speed = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            if(gamepad1.a){
                armSub.setArmAngle(30, 0.2);
                intakeSub.moveArmToPos(150, 0.2);
            }
            if(gamepad1.b){
                intakeSub.spinTake(1);
            }
            else{
                intakeSub.spinTake(0);
            }
            if(gamepad1.right_bumper){
                intakeSub.moveArmToPos(0, 1);
                armSub.setArmAngle(80, 1);
                intakeSub.grabWithClaw(0.35);
            }
            if(gamepad1.left_bumper){
                intakeSub.grabWithClaw(0.45);
            }

            chassis.drive(speed,turn);
            telemetry.addData("Intake pos", intakeSub.intakeInDeg());
            telemetry.addData("Arm Position", armSub.getPosInDegrees());
            telemetry.addData("Y axis Pwr", speed);
            telemetry.addData("X axis Pwr", turn);
            telemetry.update();
        }
    }
}
