package org.firstinspires.ftc.teamcode.ThirdRobotCode;



import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;

public class SlideSubsystem {
    static DcMotor rightSlideExtend;
    static DcMotor leftSlideExtend;

    static PIDCoefficients slidePidCoefficiients = new PIDCoefficients(Constants.SlideConstants.kp, Constants.SlideConstants.ki, Constants.SlideConstants.kd);
    static PIDFController pidfController = new PIDFController(slidePidCoefficiients, Constants.SlideConstants.kf);


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
    public static void stopSlides(){
        leftSlideExtend.setPower(0);
        rightSlideExtend.setPower(0);
    }
    public static double getSlidePos(){
        return leftSlideExtend.getCurrentPosition()* Constants.SlideConstants.GEARDIAMETER;
    }

    public static void newGoToPos(double target){
        double power = pidfController.update(getSlidePos(), target);
        leftSlideExtend.setPower(power);
        rightSlideExtend.setPower(power);

    }

    public static double slideLimit(double angle){
        if(angle<90){
            return (Constants.SlideConstants.LIMIT-Constants.SlideConstants.FORWARD_LIMIT)/Math.cos(angle*Math.PI/180)/Constants.SlideConstants.GEARDIAMETER;
        }
        else{
            return (Constants.SlideConstants.LIMIT- Constants.SlideConstants.BACKWARD_LIMIT)/Math.sin(angle*Math.PI/180)/Constants.SlideConstants.GEARDIAMETER;
        }

    }



}