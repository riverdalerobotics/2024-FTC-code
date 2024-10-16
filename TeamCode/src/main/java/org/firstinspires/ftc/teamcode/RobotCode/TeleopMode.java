package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Nicolas Teleop", group = "Linar OpMode")

public class TeleopMode {
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode(){
        Commands.scoreBucket();

    }

}
