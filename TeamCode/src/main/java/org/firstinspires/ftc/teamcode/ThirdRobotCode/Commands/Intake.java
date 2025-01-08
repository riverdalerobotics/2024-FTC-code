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
                new GoToZero(slides, arm, intake, telemetry, Constants.SlideConstants.slidesPID, Constants.ArmConstants.armPID),
                new IntakeStepTwo(intake, slides,teamColour),
                new GoToZero(slides, arm, intake, telemetry, Constants.SlideConstants.slidesPID, Constants.ArmConstants.armPID)
        );
        addRequirements(slides, arm, intake);
    }
}
