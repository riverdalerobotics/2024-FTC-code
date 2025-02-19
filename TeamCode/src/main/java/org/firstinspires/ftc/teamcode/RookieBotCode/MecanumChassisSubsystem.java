package org.firstinspires.ftc.teamcode.RookieBotCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




public class MecanumChassisSubsystem {




    //Wheels
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;


    double fr;
    double fl;
    double br;
    double bl;


    public MecanumChassisSubsystem(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        this.FL = FL;
        this.FR = FR;
        this.BR = BR;
        this.BL = BL;
    }


    public void movement(double speed, double strafe, double turn) {


    }




    public void moveRobotMech(double fwd, double strafe, double turn) {


        br = fwd - turn + strafe;
        bl = fwd + turn - strafe;
        fr = fwd - turn - strafe;
        fl = fwd + turn + strafe;




        //nerf the speed if over absolute 1
        double max = Math.max(Math.abs(fr), Math.abs(br));
        max = Math.max(max, Math.abs(fl));
        max = Math.max(max, Math.abs(bl));


        if (max > 1.0) {
            br /= max;
            fr /= max;
            bl /= max;
            fl /= max;
        }


        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);
    }
}
