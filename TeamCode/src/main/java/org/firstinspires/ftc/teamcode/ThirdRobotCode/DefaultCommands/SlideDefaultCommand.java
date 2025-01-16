package org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands;

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

public class SlideDefaultCommand extends CommandBase {
    PIDFController slidesPID;
    SlideSubsystem slides;
    PIDCoefficients pidCoefficients;
    DcMotorEx rightSlide;
    public SlideDefaultCommand(SlideSubsystem slides){
        this.slides = slides;
        this.rightSlide = slides.getRightSlideExtend();
        pidCoefficients = new PIDCoefficients(Constants.SlideConstants.kp, Constants.SlideConstants.ki, Constants.SlideConstants.kd);
        addRequirements(slides);
    }
    public void initialize() {
        slidesPID = new PIDFController(Constants.SlideConstants.kp, Constants.SlideConstants.ki, Constants.SlideConstants.kd, 0);
        slidesPID.reset();
        slidesPID.setSetPoint(slides.getSlidePos());
        slidesPID.setTolerance(0.2);
        slides.runWithOutEncoder();
    }
    @Override
    public void execute(){
        double power = -slidesPID.calculate(rightSlide.getCurrentPosition()*Constants.SlideConstants.GEARDIAMETER);
        slides.moveSlide(power);
    }

}
