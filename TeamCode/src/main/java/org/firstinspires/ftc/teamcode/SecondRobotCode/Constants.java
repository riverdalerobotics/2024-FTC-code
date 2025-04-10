package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


//TODO: what the hone is the gear ratios and also what are our encoder ticks per revolution - Sana IMPORTANT


//TODO: CHANGE EVERYTHING BACK TO FINAL CAUSE U HAVE TO HAVE IT AS NONFINAL TO ACCCESS VALUES IN FTC DASHBOARD



//TODO: DETERMING MAX AND MIN FOR THE ARM EXTENDER and the ARM SUBSYSTEM - Sana
@Config
public class Constants {
    /**
     * All constants needed for the chassis
     * */
    @Config
    static class ChassisConstants{

        /*
         * These are motor constants that should be listed online for your motors.
         */
        public static final double TICKS_PER_REV = 537.7;
        public static final double MAX_RPM = 312;

        /*
         * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
         * Set this flag to false if drive encoders are not present and an alternative localization
         * method is in use (e.g., tracking wheels).
         *
         * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
         * from DriveVelocityPIDTuner.
         */
        public static final boolean RUN_USING_ENCODER = true;
        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)); //TODO: tune thissss

        /*
         * These are physical constants that can be determined from your robot (including the track
         * width; it will be tune empirically later although a rough estimate is important). Users are
         * free to chose whichever linear distance unit they would like so long as it is consistently
         * used. The default values were selected with inches in mind. Road runner uses radians for
         * angular distances although most angular parameters are wrapped in Math.toRadians() for
         * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
         */
        public static double WHEEL_RADIUS = 2.04724; // inches
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 14.48; // in  //TODO: double check dis
        public static double WHEEL_BASE = 13.4;

        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 0.020;
        public static double kA = 0.0031;
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for you robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. All distance units are
         * inches.
         */
        public static double MAX_VEL = 60; //TODO: CHANGE DIS MAYBE, supposed to be 66
        public static double MAX_ACCEL = 50; //TODO: CHANGE DIS MAYBE, supposed to be 66
        public static double MAX_ANG_VEL = Math.toRadians(230);
        public static double MAX_ANG_ACCEL = Math.toRadians(200);

        /*
         * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
         */
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;


        public static double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }

        public static double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

    }

    /**
     * All constants needed for the the arm
     * Drive Encoder Tick per rotatioo: 312RPM  TODO:HUHHHH????
     * Arm and arm extender : 117RPM
     **/
    @Config
    public static class ArmConstants {
        public static final double Kd = 0; //TODO: find this number
        public static final double Ki = 0; //TODO: find this number
        public static final double Kp = 0; //TODO: find this number
        public static final double TOLERANCE = 0; //TODO: find this number
        public static final double GEAR_RATIO =1425.2*5; //TODO: get this number
        //double armAngle = armPivot.getCurrentPosition()*360/(1425.2*5);
        public static final double ENCODER_TICKS_PER_ROTATION = 537.7; //TODO: Find the amount of ticks per rotation
        public static final double GEAR_REDUCTION = (double) 20.0/100 ; //TODO: Find gear reduction


        public static  double ARM_ANGLE_HANDOFF = 75; //TODO: Find that up arm angle
        public static double ARM_ANGLE_PRE_INTAKE = 193;
        public static  double ARM_ANGLE_INTAKE= 209 ; //TODO: Find that down arm angle lol
        public static  double ARM_ANGLE_SLIDE_GOING_UP = 95 ; //TODO: Find that down arm angle lol
    }

    /**
     * Intake Constants needed for Intake
     * */
    @Config
    public static class SlidesConstants {

       // public static final double PULLEY_RATIO = 60;
        public static final double TOLERANCE = 0d;//find this num
        public static final double KP = 0.1;
        public static final double KI = 0d;
        public static final double KD = 0d;
        public static final double TICKS_PER_ROTATION = 537.7;
        public static final double DISTANCE_PER_ROTATION = 120; //mm

        public static final double HIGH_BASKET_POSITION = 860-3;
        public static  double LOW_BASKET_POSITION = 0;
        public static  double HANDOFF_POSITION = 0;


        public static final double ARM_NEEDS_LEAVE_POSITION = 0;


    }
    @Config
    public static class BucketConstants {

        public static  double BUCKET_SCORE_POSITION = 0.17;
        public static  double BUCKET_HANDOFF_POSITION = 0.40;

    }

    /**
     * Intake Constants needed for Intake
     * */
    @Config
    static class IntakeConstants {

        public static  double WRIST_INTAKE_POSITION = 0.988; //TODO:
        public static  double WRIST_HANDOFF_POSITION = 0.65; //TODO:


        public static  double INTAKE_SPEED = 0.9; //TODO:
        public static  double OUTAKE_SPEED = -0.2; //TODO:


    }

    /**
     * Auto constants needed for auto
     * */
    static class AutoConstants{

    }

}
