package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(IntakeSubsystem intake, SlideSubsystem slides, ArmSubsystem arm, char teamColour, MultipleTelemetry telemetry){
        addCommands(
                new ZeroIntake(intake),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, Constants.SlideConstants.slidesPID),
                new ZeroArm(arm, intake,telemetry, Constants.ArmConstants.armPID),
                new IntakeStepTwo(intake, slides,teamColour),
                new ZeroIntake(intake),
                new MoveSlides(slides, Constants.SlideConstants.ZERO, telemetry, Constants.SlideConstants.slidesPID),
                new ZeroArm(arm, intake,telemetry, Constants.ArmConstants.armPID)
                );
        addRequirements(slides, arm, intake);
    }
}
