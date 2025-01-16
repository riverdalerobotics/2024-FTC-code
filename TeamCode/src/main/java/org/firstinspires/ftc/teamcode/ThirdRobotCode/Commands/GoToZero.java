package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class GoToZero extends SequentialCommandGroup {
    IntakeSubsystem intake;
    public GoToZero(SlideSubsystem slides, ArmSubsystem arm, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients armpidf, PIDFCoefficients slidespidf){
        addCommands(
                new ZeroIntake(intake),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, slidespidf),
                new ZeroArm(arm, intake, telemetry, armpidf)
        );
        addRequirements(arm);
        addRequirements(slides);
        addRequirements(intake);
        this.intake = intake;
    }

}
