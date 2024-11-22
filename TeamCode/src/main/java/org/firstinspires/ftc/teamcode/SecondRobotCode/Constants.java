package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    /**
     * All constants needed for the chassis
     * */
    static class ChassisConstants{
//        public static final int WHEELDIAMATER = 0; //TODO: find this number
//        public static final double WHEELCIRCUMFRANCE = WHEELDIAMATER* Math.PI;
//        public static final double CHASSISWIDTH = 0; //TODO: find this number
//        public static final double CHASSISLENGTH = 0; // TODO: find this number


        /*
         * These are motor constants that should be listed online for your motors.
         */
        public static final double TICKS_PER_REV = 1; //TODO: find this number
        public static final double MAX_RPM = 1; //TODO: find this number

        /*
         * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
         * Set this flag to false if drive encoders are not present and an alternative localization
         * method is in use (e.g., tracking wheels).
         *
         * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
         * from DriveVelocityPIDTuner.
         */
        public static final boolean RUN_USING_ENCODER = false;
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
        public static double WHEEL_RADIUS = 2; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 1; // in

        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
        public static double kA = 0;
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for you robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. All distance units are
         * inches.
         */
        public static double MAX_VEL = 30;
        public static double MAX_ACCEL = 30;
        public static double MAX_ANG_VEL = Math.toRadians(60);
        public static double MAX_ANG_ACCEL = Math.toRadians(60);

        /*
         * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
         */
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


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
     * */
    static class ArmConstants {
        public static final double Kd = 0; //TODO: find this number
        public static final double Ki = 0; //TODO: find this number
        public static final double Kp = 0; //TODO: find this number
        public static final double TOLERANCE = 0; //TODO: find this number
        public static final double GEARRATIO = 1; //TODO: get this number
        public static final double ENCODERTICKPERROTATION = 0; //TODO: Find the amount of ticks per rotation
        public static final double GEARREDUCTION = 0; //TODO: Find gear reduction
        public static final double ARMANGLEUP = 0; //TODO: Find that up arm angle
        public static final double ARMANGLEDOWN = 0; //TODO: Find that down arm angle lol
    }
    public static class ArmExtender {
       public static final double WHEEDIAMITER = 1; //TODO: Find wheel diameter
    }
    /**
     * Auto constants needed for auto
     * */

    static class intakeConstants {
        public static final double MAX_INTAKE_POSITION = 0; //TODO: Find the Max Servo position for INTAKE
        public static final double MIN_INTAKE_POSITION = 0; //TODO: Find the MIN Servo position for INTAKE
        public static final double MAX_UP_POSITION = 0; //TODO: Find the ABSOLUTE_LIMIT servo position for moving it UP
        public static final double MIN_DOWN_POSITION = 0; //TODO: Find the MIN servo position for moving it DOWN
        public static final double START_INTAKE__POSITION = 0; //TODO: Find the starting position for INTAKE
        public static final double START_MIDDLE_POSITION = 0; //TODO: Find the starting position for the UP/DOWN Servo
    }
    static class AutoConstants{

    }
    /**
     * Teleop constants needed for teleop
     * */
    static class TeleopConstants{

    }
}
