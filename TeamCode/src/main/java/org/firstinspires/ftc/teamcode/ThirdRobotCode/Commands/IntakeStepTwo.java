package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class IntakeStepTwo extends ParallelRaceGroup {
    public IntakeStepTwo(IntakeSubsystem intake, SlideSubsystem slides, char teamColour){
        addCommands(
                new IntakeSpinForIntake(intake, teamColour, slides),
                new MoveSlidesForIntake(slides)
        );
        addRequirements(intake, slides);
    }
}
