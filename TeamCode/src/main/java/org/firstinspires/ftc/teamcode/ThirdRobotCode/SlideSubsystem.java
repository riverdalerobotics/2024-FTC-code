package org.firstinspires.ftc.teamcode.ThirdRobotCode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;

public class SlideSubsystem {
    static DcMotor rightSlideExtend;
    static DcMotor leftSlideExtend;

    public SlideSubsystem(DcMotor rightExtend, DcMotor leftExtend){
        this.leftSlideExtend = leftExtend;
        this.rightSlideExtend = rightExtend;

        this.leftSlideExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        // may reverse these
        // set initial position for motors
    }
    public static void moveSlide(double power){
        leftSlideExtend.setPower(power);
        rightSlideExtend.setPower(power);
    }
    public static void goToPosWithSpeed(double distance, double speed){
        leftSlideExtend.setPower(speed);
        rightSlideExtend.setPower(speed);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
        rightSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
    }
    public static void goToPos(double distance){
        leftSlideExtend.setPower(Constants.SlideConstants.SPEED);
        rightSlideExtend.setPower(Constants.SlideConstants.SPEED);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
        rightSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
    }

    public static double getSlidePos(){
        return leftSlideExtend.getCurrentPosition()* Constants.SlideConstants.GEARDIAMETER;
    }
    public static double slideLimit(double angle){
        return Constants.SlideConstants.LIMIT/Math.cos(angle*Math.PI/180)/Constants.SlideConstants.GEARDIAMETER;
    }



}