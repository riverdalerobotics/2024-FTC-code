package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class MoveSidesManually extends CommandBase {

    SlideSubsystem slides;
    MultipleTelemetry telemetry;
    double input;
    public MoveSidesManually(SlideSubsystem slides, MultipleTelemetry telemetry, double input){
        this.slides = slides;
        this.input = input;
        this.telemetry = telemetry;

        addRequirements(slides);
    }
    public void initialize(){
        slides.useRunUsingEncoders();
    }
    @Override
    public void execute(){
        super.execute();
        slides.moveSlide(input);
        telemetry.addData("Slides Power", input);
    }
    public boolean isFinished(){
        return false;
    }

}
