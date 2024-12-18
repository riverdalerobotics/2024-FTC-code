package org.firstinspires.ftc.teamcode.ThirdRobotCode;



import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;

public class SlideSubsystem {
    DcMotor rightSlideExtend;
    DcMotor leftSlideExtend;

    PIDCoefficients slidePidCoefficiients = new PIDCoefficients(Constants.SlideConstants.kp, Constants.SlideConstants.ki, Constants.SlideConstants.kd);
    PIDFController pidfController = new PIDFController(slidePidCoefficiients, Constants.SlideConstants.kf);


    public SlideSubsystem(DcMotor rightExtend, DcMotor leftExtend){
        this.leftSlideExtend = leftExtend;
        this.rightSlideExtend = rightExtend;

        this.leftSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightSlideExtend.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // may reverse these
        // set initial position for motors
    }
    public void moveSlide(double power){
        leftSlideExtend.setPower(power);
        rightSlideExtend.setPower(power);
    }
    public void goToPosWithSpeed(double distance, double speed){
        double rotation = distance/Constants.SlideConstants.GEARDIAMETER;
        leftSlideExtend.setPower(speed);
        rightSlideExtend.setPower(speed);
        leftSlideExtend.setTargetPosition((int)rotation);
        rightSlideExtend.setTargetPosition((int)rotation);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void goToPos(double distance){
        leftSlideExtend.setPower(Constants.SlideConstants.SPEED);
        rightSlideExtend.setPower(Constants.SlideConstants.SPEED);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
        rightSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
    }
    public void stopSlides(){
        leftSlideExtend.setPower(0);
        rightSlideExtend.setPower(0);
    }
    public double getSlidePos(){
        return leftSlideExtend.getCurrentPosition()*(Constants.SlideConstants.GEARDIAMETER);
    }

    public void newGoToPos(double target){
        //pidfController.setTargetPosition(target);
        //double power = pidfController.update(getSlidePos());
        //leftSlideExtend.setPower(power);
        //rightSlideExtend.setPower(power);

    }
    public void runUsingEncoders(){
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void slidesIn(){

    }
    public double slideLimit(double angle){
        if(angle<90){
            return (Constants.SlideConstants.LIMIT-Constants.SlideConstants.FORWARD_LIMIT)/Math.cos(Math.toRadians(angle))/Constants.SlideConstants.GEARDIAMETER;
        }
        else{
            return (Constants.SlideConstants.LIMIT- Constants.SlideConstants.BACKWARD_LIMIT)/Math.sin(angle*Math.PI/180)/Constants.SlideConstants.GEARDIAMETER;
        }

    }



}