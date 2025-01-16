package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class MoveSlides extends CommandBase {
    PIDFController slidesPID;
    PIDFCoefficients pidfCoefficients;
    SlideSubsystem slides;
    double setPoint;
    MultipleTelemetry telemetry;
    PIDCoefficients pidCoefficients;
    DcMotorEx rightSlide;
    double oldPower;
    double power;
    boolean hasStart = false;
    public MoveSlides(SlideSubsystem slides, double setPoint, MultipleTelemetry telemetry, PIDFCoefficients pidfCoefficients){
        this.slides = slides;
        this.setPoint = setPoint;
        this.telemetry = telemetry;
        this.pidfCoefficients = pidfCoefficients;
        this.rightSlide = slides.getRightSlideExtend();
        pidCoefficients = new PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
        addRequirements(slides);
    }
    public void initialize() {
        slidesPID = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
        slidesPID.reset();
        slides.runWithOutEncoder();
        slidesPID.setSetPoint(setPoint);
        slidesPID.setTolerance(Constants.SlideConstants.TOLERANCE);
        slides.runWithOutEncoder();

    }
    @Override

    public void execute(){


        power = -slidesPID.calculate(rightSlide.getCurrentPosition()*Constants.SlideConstants.GEARDIAMETER, setPoint);

        slides.moveSlide(power);
        telemetry.addData("SLIDES POS AS GIVEN BY PID", slides.getSlidePos());
        telemetry.addData("Slides Power", power);
        telemetry.addData("Error", slidesPID.getPositionError());
        }
    public boolean isFinished(){
        return slidesPID.atSetPoint();
    }
    public void end(){
        slidesPID.reset();
        slides.moveSlide(0);
    }

}
