package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCode.TeleopMode;
@TeleOp(name = "Test OpMode", group = "Linear OpMode")
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor testMotor;
        testMotor = hardwareMap.get(DcMotor.class, "TestMotor");
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while(opModeIsActive()){
            double power = gamepad1.left_stick_y;
            if(gamepad1.a){
                testMotor.setPower(-0.3);
                testMotor.setTargetPosition((int)(-100*537.6/(38.2*Math.PI)));
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            telemetry.addData("Motor Position", testMotor.getCurrentPosition()/(537.6)*38.2*Math.PI);
            telemetry.update();
        }
    }
}
