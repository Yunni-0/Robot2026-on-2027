// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.stream.Stream;

import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.studica.frc.AHRS;
//import com.studica.frc.AHRS.NavXComType;

import org.wpilib.hardware.hal.SimDouble;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.Units;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.simulation.SimDeviceSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drive extends SubsystemBase {
	private static Drive s_theDrive;

	private final SwerveModule m_frontLeft = new SwerveModule(0, kFrontLeftCANCoderPort, kFrontLeftDrivePort,
			kFrontLeftSteerPort, true);
	private final SwerveModule m_frontRight = new SwerveModule(1, kFrontRightCANCoderPort, kFrontRightDrivePort,
			kFrontRightSteerPort, false);
	private final SwerveModule m_backLeft = new SwerveModule(2, kBackLeftCANCoderPort, kBackLeftDrivePort,
			kBackLeftSteerPort, true);
	private final SwerveModule m_backRight = new SwerveModule(3, kBackRightCANCoderPort, kBackRightDrivePort,
			kBackRightSteerPort, false);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	//private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
	//private final SimDouble m_gyroSim;
	private final SysIdRoutine m_sysidRoutine;

	private final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/Pose", Pose2d.struct).publish();
	private final StructPublisher<ChassisVelocities> m_currentChassisVelocitiesPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/Chassis Speeds", ChassisVelocities.struct).publish();
	private final StructArrayPublisher<SwerveModuleVelocity> m_targetModuleStatePublisher = NetworkTableInstance
			.getDefault().getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleVelocity.struct)
			.publish();
	private final StructArrayPublisher<SwerveModuleVelocity> m_currentModuleStatePublisher = NetworkTableInstance
			.getDefault().getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleVelocity.struct)
			.publish();

	private final PIDController m_orientationController = new PIDController(kP, kI, kD);

	private NeutralModeValue coastMode = NeutralModeValue.Coast;

	/** Creates a new DriveSubsystem. */
	public Drive() {
		if (s_theDrive == null) {
			s_theDrive = this;
		} else {
			throw new Error("Drive already instantiated");
		}
		m_orientationController.enableContinuousInput(-Math.PI, Math.PI);
		// Adjust ramp rate, step voltage, and timeout to make sure robot doesn't
		// collide with anything
		m_sysidRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(Units.Volts.of(2.5).div(Units.Seconds.of(1)), null, Units.Seconds.of(3)),
				new SysIdRoutine.Mechanism(volt -> doModuleX(
						module -> module
								.setModuleState(new SwerveModuleVelocity(volt.magnitude(), new Rotation2d(Math.PI / 2))),
						SwerveModuleVelocity[]::new), null, this));
		//m_gyro.zeroYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
		if (RobotBase.isSimulation()) {
			//m_gyroSim = new SimDeviceSim("navX-Sensor", m_gyro.getPort()).getDouble("Yaw");
		} else {
			//m_gyroSim = null;
		}
	}

	public static Drive getDrive() {
		return s_theDrive;
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public static Rotation2d getHeading() {
		//TODO: Wait for Studica libs
		//return s_theDrive.m_gyro.getRotation2d();
		return Rotation2d.ZERO;
	}

	/**
	 * Resets drive encoders to zero.
	 */
	private static void resetEncoders() {
		doModuleX(SwerveModule::resetDriveEncoder, Boolean[]::new);
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public static Pose2d getPose() {
		return s_theDrive.m_odometry.getPose();
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public static SwerveModulePosition[] getModulePositions() {
		return doModuleX(SwerveModule::getModulePosition, SwerveModulePosition[]::new);
	}

	private static <T> T[] doModuleX(Function<SwerveModule, T> function, IntFunction<T[]> constructor) {
		return Stream.of(s_theDrive.m_frontLeft, s_theDrive.m_frontRight, s_theDrive.m_backLeft, s_theDrive.m_backRight)
				.map(function).toArray(constructor);
	}

	/**
	 * Calculates module states from a chassis speeds.
	 * 
	 * @param speeds The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds is field relative.
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	private static SwerveModuleVelocity[] calculateModuleStates(ChassisVelocities speeds, boolean isRobotRelative) {
		if (Constants.kLogging) {
			Telemetry.log("Drive/Heading", getHeading().getDegrees());
			Telemetry.log("Drive/Robot Relative", isRobotRelative);
		}
		if (!isRobotRelative)
			speeds = speeds.toFieldRelative(getHeading());
		speeds = speeds.discretize(0.03);
		SwerveModuleVelocity[] states = s_theDrive.m_kinematics.toSwerveModuleVelocities(speeds);
		states = SwerveDriveKinematics.desaturateWheelVelocities(states, 1);
		Double[] moduleAngles = doModuleX(SwerveModule::getModuleAngle, Double[]::new);
		for (int i = 0; i < states.length; i++) // Optimize target module states
			states[i] = states[i].optimize(Rotation2d.fromDegrees(moduleAngles[i]));
		return states;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds The chassis speeds.
	 */
	private static void setModuleStates(SwerveModuleVelocity[] states) {
		s_theDrive.m_targetModuleStatePublisher.set(states);
		doModuleX(module -> module.setModuleState(states[module.getIndex()]), SwerveModuleVelocity[]::new);
	}

	public static void stop() {
		setModuleStates(calculateModuleStates(new ChassisVelocities(), false));
	}

	public static void setCoastMode(NeutralModeValue mode) {
		s_theDrive.coastMode = mode;
		doModuleX(module -> module.setNeutralMode(mode), NeutralModeValue[]::new);
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}. Useful
	 * for updating subsystem-specific state.
	 */
	@Override
	public void periodic() {
		SwerveModuleVelocity[] states = doModuleX(SwerveModule::getModuleState, SwerveModuleVelocity[]::new);
		m_currentModuleStatePublisher.set(states);
		var speeds = m_kinematics.toChassisVelocities(states);
		m_currentChassisVelocitiesPublisher.set(speeds);
		if (RobotBase.isSimulation())// TODO: Use SysId to get feedforward model for rotation
			/*m_gyroSim.set(
					-Math.toDegrees(speeds.omega * TimedRobot.DEFAULT_PERIOD) + m_gyro.getYaw());*/
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
	}

	public static void toggleCoastMode() {
		setCoastMode(switch (s_theDrive.coastMode) {
			case Coast -> NeutralModeValue.Brake;
			case Brake -> NeutralModeValue.Coast;
		});
	}

	/**
	 * Method for making the robot drive using speeds
	 * 
	 * @param forwardSpeed
	 * @param strafeSpeed
	 * @param rotation
	 * @param isRobotRelative
	 */
	public static void drive(double forwardSpeed, double strafeSpeed, double rotation, boolean isRobotRelative) {
		setModuleStates(
				calculateModuleStates(new ChassisVelocities(forwardSpeed, strafeSpeed, rotation), isRobotRelative));
	}

	/**
	 * Turns all steer motors to a specified angle.
	 * 
	 * @param angle The desired angle (in degrees).
	 */
	public void turnSteerToAngle(double angle) {
		SwerveModuleVelocity state = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(angle));
		setModuleStates(new SwerveModuleVelocity[] { state, state, state, state });
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public static void resetHeading() {
		//s_theDrive.m_gyro.zeroYaw();
	}

	public static void resetOdometry(Pose2d pose) {
		s_theDrive.m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public static Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return s_theDrive.m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public static Command sysidDynamic(SysIdRoutine.Direction direction) {
		return s_theDrive.m_sysidRoutine.dynamic(direction);
	}
}