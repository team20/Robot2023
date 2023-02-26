package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public final class Constants {
	public static final class ArduinoConstants {
		/**
		 * The I2C address of the Arduino as defined by the address passed into
		 * Wire.begin() in the Arduino code
		 */
		public static final int kAddress = 0x18;
	}

	public static final class ArmConstants {
		// Preset angles for arm
		/** The lower and upper arm angles for the arm to be in the high position */
		public static final double[] kHighAngles = { 52, 153 };
		public static final double[] kHighIntermediateAngles = { (90 + (kHighAngles[0]))/2, kHighAngles[1] };

		/**
		 * The lower and upper arm angles for the arm to be flipped over in the medium
		 * position
		 */
		public static final double[] kHighBackAngles = { 109, 236 };

		public static final double[] kMediumBackAngles = { 87, 280 };

		/**
		 * The lower and upper arm angles for the arm to be forwards in the medium
		 * position
		 */
		public static final double[] kMediumForwardAngles = { 84, 98 };
		/** The lower and upper arm angles for the arm to be in the low position */
		public static final double[] kLowAngles = { 90, 40 };
		/** The lower and upper arm angles for the arm to be in the frame pocket */
		public static final double[] kPocketAngles = { 90, 33 };
		/**
		 * The lower and upper arm angles for the arm to be in the intermediate position
		 * needed to prevent the arm from going over the height limit
		 */
		public static final double[] kIntermediateAngles = { 50, 220 };

		// TODO remeasure arm lengths
		/** Length of lower arm length in inches */
		public static final double kLowerArmLength = 33;
		/** Length of upper arm length in inches */
		public static final double kUpperArmLength = 43.5;


		// TODO check arm speed multipliers
		/**
		 * A joystick input multiplier to control how fast the arm moves relative to how
		 * much the joystick is being moved. Used for moving the arm position(x and y
		 * coordinates)
		 */
		public static final double kCartesianSpeedSensitivity = 1.5;
		/**
		 * A joystick input multiplier to control how fast the arm motors spin relative
		 * to how much the joystick is being moved. Used for spinning the arm motors
		 * manually
		 */
		public static final double kArmMotorSpeedSensitivity = 0.4;

		// TODO Check allowable error, and max angle and height values
		/**
		 * Allowable difference in degrees between the target arm angle and the current
		 * arm angle
		 */
		public static final double kAllowedDegreesError = 4;
		/** Smallest angle the lower arm can go */
		public static final double kLowerArmMinAngle = 45;
		/** Maximum angle the lower arm can go */
		public static final double kLowerArmMaxAngle = 135;
		/** Smallest angle the lower arm can go */
		public static final double kUpperArmMinAngle = 15;
		/** Maximum angle the lower arm can go */
		public static final double kUpperArmMaxAngle = 290;
		/**
		 * Maximum height in inches the arm can reach.
		 * <p>
		 * The max height for the robot is 6 feet, 6 inches, or 78 inches.
		 * <p>
		 * The base of the robot is 10.5 inches, leaving 67.5 inches left for the arm
		 */
		public static final double kMaxHeight = 80;
		// TODO adjust offsets
		/**
		 * Number of degrees the lower arm encoder output needs to be offset so it reads
		 * 0 degrees in our zero position
		 */
		public static final double kLowerEncoderZeroOffset = 29.967;

		/**
		 * Number of degrees the upper arm encoder output needs to be offset so it reads
		 * 0 degrees in our zero position
		 */
		// TODO encoder offsets
		public static final double kUpperEncoderZeroOffset = 356.282;
		public static final int kUpperMotorID = 8;
		public static final int kLowerMotorID = 6;
		public static final int kLowerMotor2ID = 7;
		// TODO evaluate current limits
		public static final int kSmartCurrentLimit = 20;
		public static final int kPeakCurrentLimit = 30;
		public static final int kPeakCurrentDurationMillis = 100;
		public static boolean kLowerArmMotor2Oppose = true;
		// TODO PIDS
		public static final double kLowerArmP = 0.06000;
		public static final double kLowerArmI = 0.000;
		public static final double kLowerArmD = 0;
		public static final double kLowerArmIz = 5;
		public static final double kLowerArmFF = 0.0;
		public static final boolean kLowerInvert = true;
		public static final double kUpperArmP = 0.1;//0.25
		public static final double kUpperArmI = 0.000;
		public static final double kUpperArmD = 0;
		public static final double kUpperArmIz = 5;
		public static final double kUpperArmFF = 0.0;
		public static final boolean kUpperInvert = false;
		// TODO set this back to one?
		public static final double kMinOutputLower = -1;
		public static final double kMaxOutputLower = 1;
		public static final double kMinOutputUpper = -1;
		public static final double kMaxOutputUpper = 1;
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			public static final int kSquare = 1;
			public static final int kX = 2;
			public static final int kCircle = 3;
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kLeftTrigger = 7;
			public static final int kRightTrigger = 8;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {

		// TODO recheck inversions
		public static final int kFrontLeftID = 2;
		public static final boolean kFrontLeftInvert = false;
		public static final int kBackLeftID = 4;
		public static final boolean kBackLeftOppose = false;

		public static final int kFrontRightID = 3;
		public static final boolean kFrontRightInvert = true;
		public static final int kBackRightID = 5;
		public static final boolean kBackRightOppose = false;

		// TODO re-evaluate current limits
		public static final int kSmartCurrentLimit = 55;
		public static final double kPeakCurrentLimit = 65;
		public static final int kPeakCurrentDurationMillis = 0;

		// TODO PIDS
		public static final double kP = .14;// 0.198;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;

		// TODO do we need slot id
		public static final int kSlotID = 0;

		// TODO do we use this
		public static final double kFineTurningSpeed = .1;

		// navX stuff
		public static final SPI.Port kGyroPort = SPI.Port.kMXP;
		// TODO is gyro truly reversed?
		public static final boolean kGyroReversed = true;

		// TODO CHANGE ADJUST PIDS
		public static final double kTurnP = 0.004; // was 0.005
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnTolerance = 0.5;

		// TODO unused constants
		public static final double ksVolts = 0.196;
		public static final double kvVoltSecondsPerMeter = 2.15;
		public static final double kaVoltSecondsSquaredPerMeter = .53;

		// Horizontal distance between the wheels
		public static final double kTrackwidthMeters = Units.inchesToMeters(20.5);

		// TODO remove unused
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackwidthMeters);

		// TODO see which ones of these are used and which are not
		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = .5;
		public static final double kMaxRotSpeedMetersPerSecond = 1;
		public static final double kWheelDiameterMeters = Units.inchesToMeters(5.75);
		public static final double kGearRatio = 9.4;
		public static final double kTurningMultiplier = .3;
		public static final double kQuickStopThreshold = .2;
		public static final double kQuickStopAlpha = .1;
		public static final double kBackupDistance = Units.feetToMeters(2);
		public static final double kRampRate = .5;// 1?
		public static final double kSpeedLimitFactor = 1.0;
		public static final boolean kLeftSensorPhase = true; // TODO these are totally arbitrary right now and need to
																// // be checked
		public static final boolean kRightSensorPhase = false;
		public static final boolean kEnableVoltageComp = true;
		public static final double kVoltageComp = 12;
		public static final double kEncoderCounts = 42;
		/**
		 * Converts native encoder units(revolutions) to meters
		 * <p>
		 * Native units are in revolutions, 1 / gearRatio gives us how many revolutions
		 * the wheel has turned, and multiplying that by the wheel circumference(pi
		 * times the wheel diameter) gives the distance the robot has moved in meters
		 */
		public static final double kEncoderPositionConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters;
		/**
		 * Converts native encoder units(RPM) to meters per minute
		 * <p>
		 * Native units are in revolutions per minute, 1 / gearRatio gives us how many
		 * revolutions the wheel has turned per minute, and multiplying that by the
		 * wheel circumference(pi times the wheel diameter) gives wheel velocity in
		 * meters per minute, and dividing that by 60 gives wheel velocity in meters per
		 * second. It's in meters per second because that's the unit
		 * DifferentialDriveWheelSpeeds uses
		 */
		public static final double kEncoderVelocityConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters / 60;
		// TODO PIDS
		public static final double kBalanceP = 0.0085;
		public static final double kBalanceI = 0.0001;
		public static final double kBalanceD = 0.001;


	}

	public static final class GripperConstants {
		public static final double kGripperOpenPosition = 0.5;// TODO is this correct
		public static final boolean kInvert = false;
		public static final int kGripperID = 9;
		public static final int kSmartCurrentLimit = 15;// TODO evaluate current limit (37 for gripper, 15 for wheel)
		public static final double kCloseTime = 3000; // TODO: change as needed
		public static final double kMovePower = 0.6; // TODO check (0.2 for gripper, 0.4 for wheel gripper)
		public static final double kHoldPower = 0.1; // TODO: change as needed
	}

	public static final class LimelightConstants { 
		//TODO make real numbers
		public static final double zTolerance = 0.2; 
		public static final double xTolerance = 0.2;
		public static final int kRollingMedianSize = 10;
		public static final double kSlowDownDistance = 0.5;
		public static final double kSlowDownDistanceSquared = kSlowDownDistance*kSlowDownDistance;
		public static final double kSpeed = 0.35;
	}

}
