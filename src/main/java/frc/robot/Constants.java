package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public final class Constants {
	public static final class GripperConstants {
		public static final double kGripperOpenPosition = 0.5;
		public static final boolean kFrontLeftInvert = false;
		public static final int kPort = 4;
		public static final double kP = 0.0003; // have to figure out constants later
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 5;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 20;
		public static final double kWinchEncoderZeroOffset = 110.2;
		public static final double kCloseTime = 1000; // TODO: change as needed
		public static final double kHoldPower = .1; // TODO: change as needed
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;
		public static final int kOpenLimitSwitchPort = 0; // change port
		public static final int kCloseLimitSwitchPort = 0; // change port
	}

	public static final class ArmConstants {
		// Preset angles for arm
		// Arm is foward
		public static final double[] kHighAngles = { 50, 160 };
		// Arm is backward
		public static final double[] kMediumBackAngles = { 90, 270 };
		// Arm is forward
		public static final double[] kMediumForwardAngles = { 90, 90 };
		// Arm is foward
		public static final double[] kLowAngles = { 90, 40 };
		// Angles for putting the arm in the pocket
		public static final double[] kPocketAngles = { 105, 15 };
		// Transition position
		public static final double[] kIntermediateAngles = { 50, 220 };
		// Recheck when robot is assembled
		/** Length of lower arm length in inches */
		public static final double kLowerArmLength = 32.5;
		/** Length of upper arm length in inches */
		public static final double kUpperArmLength = 40;
		/**
		 * A joystick input multiplier to control how fast the arm moves relative to how
		 * much the joystick is being moved
		 */
		public static final double kArmMovementSpeedMultiplier = 1.5;
		/**
		 * A joystick input multipler to control how fast the arm motors spin relative
		 * to how much the joystick is being moved
		 */
		public static final double kManualArmMovementSpeedMultiplier = 0.5;
		public static final int kCountsPerRevolution = 42;
		/**
		 * Allowable difference in degrees between the target arm angle and the current
		 * arm angle
		 */
		public static final double kAllowedDegreesError = 4;
		public static final double kMinEncoderValue = 0.0;
		public static final double kMaxEncoderValue = 42.0;
		/** Smallest angle the lower arm can go */
		public static final double kLowerArmMinAngle = 45;
		/** Maximum angle the lower arm can go */
		public static final double kLowerArmMaxAngle = 135;
		/** Smallest angle the lower arm can go */
		public static final double kUpperArmMinAngle = 15;
		/** Maximum angle the lower arm can go */
		public static final double kUpperArmMaxAngle = 270;
		/** Maximum height in inches the arm can go to */
		public static final double kMaxHeight = 12;
		/**
		 * Number of degrees the lower arm encoder output needs to be offset so it reads
		 * 0 degrees in our zero position
		 */
		public static final double kLowerEncoderZeroOffset = 110.2;
		/**
		 * Number of degrees the upper arm encoder output needs to be offset so it reads
		 * 0 degrees in our zero position
		 */
		public static final double kUpperEncoderZeroOffset = 248.9;
		public static final int kUpperMotor = 1;
		public static final int kLowerMotor = 2;
		public static final int kLowerMotor2 = 3;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 20;
		public static final int kPeakCurrentLimit = 30;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kLowerArmP = 0.0070;
		public static final double kLowerArmI = 0.0001;
		public static final double kLowerArmD = 0;
		public static final double kLowerArmIz = 5;
		public static final double kLowerArmFF = 0.0;
		public static final double kUpperArmP = 0.0070;
		public static final double kUpperArmI = 0.0001;
		public static final double kUpperArmD = 0;
		public static final double kUpperArmIz = 5;
		public static final double kUpperArmFF = 0.0;
		public static final double kMinOutput = -.4;
		public static final double kMaxOutput = .4;
		public static final int kSlotID = 0;
		public static final double kMaxAcel = 0;
		public static final double kMaxVelocity = 0;
		public static final double kMinVelocity = 0;
		public static final double kMinPosition = 0;
		public static final double kInPosition = 0;
		public static final double kOutPosition = 0;
		public static final double kBounceDownPosition = 0;
		public static final double kBounceUpPosition = 0;
		public static final double kBounceTime = 0;
	}

	public static final class ArduinoConstants {
		public static final int kAddress = 2;
		public static final double kDistanceP = 0;
		public static final double kDistanceI = 0;
		public static final double kDistanceD = 0;
		public static final int kDistanceSetpoint = 0;

		public static final double kAngleP = 0;
		public static final double kAngleI = 0;
		public static final double kAngleD = 0;
		public static final int kAngleSetpoint = 0;

		public static final int kReadTargetInView = 0;
		public static final int[] kReadXValue = { 1, 2, 3 };
		public static final int[] kReadDistance = { 4, 5, 6 };

		public static final int kWriteMainLEDMode = 0;
		public static final int kWriteMainLEDValue = 1;
		public static final int kWriteShooterLEDMode = 2;
		public static final int kWriteShooterLEDValue = 3;
		public static final int kWriteClimberLEDMode = 2;
		public static final int kWriteClimberLEDValue = 3;

		public static final class LEDModes {
			public static final byte kReset = 0;
			public static final byte kOff = 1;
			public static final byte kSolid = 2;
			public static final byte kChasing = 3;
			public static final byte kTheaterLights = 4;
			public static final byte kRedGreenGradient = 5;
			public static final byte kBlueGreenGradient = 6;
			public static final byte kBackForthTimer = 7;
		}

		public static final class LEDColors {
			public static final byte kOff = 0;
			public static final byte kRed = 1;
			public static final byte kOrange = 2;
			public static final byte kYellow = 3;
			public static final byte kGreen = 4;
			public static final byte kBlue = 5;
			public static final byte kPurple = 6;
			public static final byte kWhite = 7;
		}
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

		public static final class PS4Axis {
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

		// TODO CHANGE ALL OF THESE
		public static final int kFrontLeftPort = 13;
		public static final boolean kFrontLeftInvert = true;
		public static final int kBackLeftPort = 12;
		public static final boolean kBackLeftOppose = false;

		public static final int kFrontRightPort = 11;
		public static final boolean kFrontRightInvert = false;
		public static final int kBackRightPort = 10;
		public static final boolean kBackRightOppose = false;

		public static final int kSmartCurrentLimit = 55;
		public static final double kPeakCurrentLimit = 65;
		public static final int kPeakCurrentDurationMillis = 0;
		public static final double kP = .14;// 0.198;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;

		public static final double kFineTurningSpeed = .1;

		// navX stuff
		public static final SPI.Port kGyroPort = SPI.Port.kMXP;
		public static final boolean kGyroReversed = true;

		// TODO CHANGE ALL OF THESE
		public static final double kTurnP = 0.0125; // was 0.005
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnTolerance = 0.5;
		public static final double ksVolts = 0.196;
		public static final double kvVoltSecondsPerMeter = 2.15;
		public static final double kaVoltSecondsSquaredPerMeter = .53;
		// TODO change the trackwidth to match our robot - trackwidth = horizontal
		// distance between the wheels
		public static final double kTrackwidthMeters = 0.7815245428457417;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackwidthMeters);

		// TODO change
		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = .5;
		public static final double kMaxRotSpeedMetersPerSecond = 1;
		public static final double kWheelDiameterMeters = 4;
		public static final double kGearRatio = 7;
		public static final double kTurningMultiplier = .45;
		public static final double kQuickStopThreshold = .2;
		public static final double kQuickStopAlpha = .1;
		public static final double kBackupDistance = Units.feetToMeters(2);
		public static final double kRampRate = .1;// 1?
		public static final double kSpeedLimitFactor = .5;
		public static final boolean kLeftSensorPhase = true; // TODO these are totally arbitrary right now and need to
																// // be checked
		public static final boolean kRightSensorPhase = false;
		public static final boolean kEnableVoltageComp = true;
		public static final double kVoltageComp = 12;
		public static final double kEncoderCounts = 4096;
		public static final double kEncoderPositionConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters;
		public static final double kEncoderVelocityConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters * 60;

	}

	public static final class LimelightConstants { // TODO: tune PID loop
		public static final double kDisP = 0.02;
		public static final double kDisI = 0;
		public static final double kDisD = 0;
		public static final double kTurnP = 0.03;
		public static final double kTurnI = 0.0000;
		public static final double kTurnD = 0.0;
		public static final double kTurnTolerance = 2; // TODO: this is the amount of error that is considered okay
														// because obviously we can't get perfectly to the setpoint
		public static final double kDistanceTolerance = 0.5;
		public static final double kCameraHeight = 22.5; // TODO: get height once mounted
		public static final double kCameraAngle = 25.453;// 29.8394991; //TODO: get angle once mounted
		public static final double kTargetHeight = 104; // TODO: get height of target (inches)
		public static final double kRefreshRate = 0.01111; // matches with the max of 90 frames/second from the
															// limelight
		public static final int kRollingAverageSize = 10; // TODO: change, experiment
	}

}
