package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.SlideSubsystem;

public class IntakeStepTwoButFancy extends ParallelDeadlineGroup {
    public IntakeStepTwoButFancy(IntakeSubsystem intake, SlideSubsystem slides, char teamColour){
        super(new IntakeSpinForIntake(intake, teamColour, slides),
                new MoveSlidesForIntake(slides));
        addRequirements(intake, slides);

    }

}
