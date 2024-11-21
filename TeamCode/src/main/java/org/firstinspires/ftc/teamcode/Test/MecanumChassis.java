package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SecondRobotCode.ArmSubsytem;
import org.firstinspires.ftc.teamcode.SecondRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.SecondRobotCode.IntakeSubsystem;

@TeleOp(name="Sana Mecanum", group="Linear OpMode")
public class MecanumChassis extends LinearOpMode {

    public DcMotor motorLeftF;
    public DcMotor motorRightF;
    public DcMotor motorRightB;
    public DcMotor motorLeftB;

    ChassisSubsystem chassis;
    ArmSubsytem arm;
    IntakeSubsystem intake;
    org.firstinspires.ftc.teamcode.SecondRobotCode.armExtender armExtender;


    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.get(DcMotor.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotor.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotor.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotor.class, "motorLeftB");
        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB);
        // arm = new ArmSubsystem();
        waitForStart();
        double fwdPwr;
        double strafePwr;
        double rotationPwr;
        while (opModeIsActive()){

            fwdPwr = -gamepad1.left_stick_y;
            strafePwr = -gamepad1.left_stick_x;
            rotationPwr = -gamepad1.right_stick_x;
            chassis.moveMechChassis(fwdPwr, strafePwr, rotationPwr);

            telemetry.addData("Status", "wobot is on :3");
           // moveMecChassis();
            telemetry.update();
        }

        }
    }


