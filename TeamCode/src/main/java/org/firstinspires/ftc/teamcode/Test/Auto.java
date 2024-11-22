package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Hot Dog Wheels  TEST Auto", group = "Linear OpMode")
public class Auto extends LinearOpMode{

    private DcMotor motorLeftF;
    //Right Front motor is connected to port 0
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    //Right Back Motor is connected to port 1
    private  DcMotor motorLeftB;


    double leftFront;
    double leftBack;
    double rightFront;
    double rightBack;

    final double MAX_AUTO_SPEED = 0.4;
    final double MAX_AUTO_TURN = 0.3;
    final double MAX_AUTO_STRAFE = 0.3;

    double Encoder_ticksPerRevolution = 480;
    double wheelCircumference = 10*Math.PI;
    double ticksPerCm = Encoder_ticksPerRevolution/ wheelCircumference;
    double centimeters = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftF = hardwareMap.get(DcMotor.class, "motorLeftF");
        motorRightF = hardwareMap.get(DcMotor.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotor.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotor.class, "motorLeftB");
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            motorRightB.setDirection(DcMotor.Direction.REVERSE);
            motorRightF.setDirection(DcMotor.Direction.REVERSE);

            motorLeftF.setTargetPosition(400);
            motorLeftB.setTargetPosition(10000);
            motorRightB.setTargetPosition(10000);
            motorRightF.setTargetPosition(10000);
            motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //motorLeftF.setPower(0.3);
            //motorLeftB.setPower(0.3);
            //motorRightB.setPower(0.3);
           // motorRightF.setPower(0.3);
            motorRightB.setPower(0.1);
            motorLeftB.setPower(0.1);
            motorLeftF.setPower(0.1);
            motorRightF.setPower(0.1);

            //twwwelemetry.addData(motorRightB.getCurrentPosition());


        }
        // Once the Motors reach their destination the power is set back 0
        //motorLeftF.setPower(0);
        //motorLeftB.setPower(0);
        //motorRightB.setPower(0);
        //motorRightF.setPower(0);
    }



        //chassis.moveRobotMecanum(0, 0, 0);
    }
