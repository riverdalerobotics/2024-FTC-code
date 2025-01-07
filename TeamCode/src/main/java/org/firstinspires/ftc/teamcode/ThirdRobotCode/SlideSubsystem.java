package org.firstinspires.ftc.teamcode.ThirdRobotCode;



import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;


public class SlideSubsystem extends SubsystemBase {
    DcMotorEx rightSlideExtend;
    DcMotorEx leftSlideExtend;



    PIDCoefficients slidePidCoefficiients = new PIDCoefficients(Constants.SlideConstants.kp, Constants.SlideConstants.ki, Constants.SlideConstants.kd);
    PIDFController pidfController = new PIDFController(slidePidCoefficiients, Constants.SlideConstants.kf);

    MultipleTelemetry telemetry;
    public SlideSubsystem(DcMotorEx rightExtend, DcMotorEx leftExtend, MultipleTelemetry telemetry){
        this.leftSlideExtend = leftExtend;
        this.rightSlideExtend = rightExtend;
        this.telemetry = telemetry;
        this.leftSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightSlideExtend.setDirection(DcMotorSimple.Direction.FORWARD);

//        leftSlideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // may reverse these
        // set initial position for motors
    }
    public void moveSlide(double power){
        leftSlideExtend.setPower(power);
        rightSlideExtend.setPower(power);
    }
    public void useRunUsingEncoders(){
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void goToPosWithSpeed(double distance, double speed){
        useRunUsingEncoders();
        double rotation = distance/Constants.SlideConstants.GEARDIAMETER;
        leftSlideExtend.setPower(0);
        rightSlideExtend.setPower(speed);
        leftSlideExtend.setTargetPosition((int)rotation);
        rightSlideExtend.setTargetPosition((int)rotation);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void goToPos(double distance){
        leftSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
        rightSlideExtend.setTargetPosition((int)distance/(int)Constants.SlideConstants.GEARDIAMETER);
        leftSlideExtend.setPower(Constants.SlideConstants.SPEED);
        rightSlideExtend.setPower(Constants.SlideConstants.SPEED);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void stopSlides(){
        leftSlideExtend.setPower(0);
        rightSlideExtend.setPower(0);
    }
    public void goToCurrentPos(){
        rightSlideExtend.setTargetPosition(rightSlideExtend.getCurrentPosition());
        leftSlideExtend.setTargetPosition(leftSlideExtend.getCurrentPosition());
        leftSlideExtend.setPower(0.1);
        rightSlideExtend.setPower(0.1);
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getSlidePos(){
        return rightSlideExtend.getCurrentPosition()*(Constants.SlideConstants.GEARDIAMETER);
    }
    public DcMotorEx getRightSlideExtend(){
        return rightSlideExtend;
    }

    public DcMotorEx getLeftSlideExtend() {
        return leftSlideExtend;
    }

    public void newGoToPos(double target){
        //pidfController.setTargetPosition(target);
        //double power = pidfController.update(getSlidePos());
        //leftSlideExtend.setPower(power);
        //rightSlideExtend.setPower(power);

    }
    public void runWithOutEncoder(){
        leftSlideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    @Override
    public void periodic(){
        super.periodic();
        telemetry.addData("Current Slide Positiom", getSlidePos());
        telemetry.addData("THE POWER OF SLIDES", rightSlideExtend.getPower());
    }



}