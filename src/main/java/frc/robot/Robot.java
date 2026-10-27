// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.framework.TimedRobot;
import org.wpilib.telemetry.Telemetry;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.button.CommandDualSenseController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
	public static boolean isCompBot = false;
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final CommandDualSenseController m_driverController = new CommandDualSenseController(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandDualSenseController m_operatorController = new CommandDualSenseController(
			Constants.ControllerConstants.kOperatorControllerPort);
	private final Aim m_aim = new Aim.Linear();

	{
		new Drive();
		/*
		 * new Shooter();
		 * Turret.create();
		 * Hood.create();
		 */

		new Intake();
	}

	public Robot() {

	}

	private void bindControls() {
		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						(() -> m_driverController.getR2() - m_driverController.getL2()),
						m_driverController.getController()::getCreateButton));
		m_driverController.options().debounce(.1).onTrue(new DriveCommands.ResetHeading());

		m_driverController.L1().whileTrue(new IntakeCommands.SpinIntake(1.0));
		m_driverController.R1().whileTrue(new IntakeCommands.StopIntake());
		/*
		 * Turret.getTurret().setDefaultCommand(
		 * new TurretCommands.RunToAngleHardwareSignal(m_operatorController::getLeftX,
		 * m_operatorController::getLeftY));
		 * m_operatorController.L1().whileTrue(new TurretCommands.RunAtPower(-.1, 0));
		 * m_operatorController.R1().whileTrue(new TurretCommands.RunAtPower(.1, 0));
		 * m_operatorController.triangle().toggleOnTrue(
		 * new ShooterCommands.RunAtDPadRPM(this, m_operatorController.povRight(),
		 * m_operatorController.povLeft()));
		 * m_operatorController.povDown().whileTrue(new HoodCommands.RunAtPower(-.1,
		 * 0));
		 * m_operatorController.povUp().whileTrue(new HoodCommands.RunAtPower(.1, 0));
		 */
		/*
		 * m_operatorController.square().onTrue(new IntakeCommands.Spin(.1));
		 * m_operatorController.circle().onTrue(new IntakeCommands.ExtendArmCommand());
		 * m_operatorController.cross().onTrue(new IntakeCommands.RetractArmCommand());
		 */
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();
		if (Constants.kLogging) {
			Telemetry.log("Scheduler", m_scheduler);
		}
	}

	@Override
	public void autonomousInit() {
		m_scheduler.cancelAll();
		/*
		 * m_scheduler.schedule(
		 * Commands.parallel(
		 * Commands.sequence(
		 * new TurretCommands.RunToAngleHardware(45), Commands.waitSeconds(1),
		 * new TurretCommands.RunToAngleHardware(225), Commands.waitSeconds(1),
		 * new TurretCommands.RunToAngleHardware(45), Commands.waitSeconds(1),
		 * new TurretCommands.RunToAngleHardware(225)),
		 * new ShooterCommands.RunAtDynamicRPM(2400).withTimeout(40)));
		 */

		m_scheduler.schedule(
				new SequentialCommandGroup(
						new DriveCommands.DriveDistance(-1.5),
						new DriveCommands.DriveDistance(1.5),
						new DriveCommands.DriveDistance(-1.5),
						new DriveCommands.DriveDistance(1.5),
						new DriveCommands.DriveDistance(-1.5),
						new DriveCommands.DriveDistance(1.5)));

		// new DriveCommands.TurnSteerToAngle(0),
		// new WaitCommand(1),
		// new DriveCommands.TurnSteerToAngle(45),
		// new WaitCommand(1),
		// new DriveCommands.TurnSteerToAngle(0)));
		// m_scheduler.schedule(m_aim.getAimCommand(13.5));
	}

	@Override
	public void teleopInit() {
		m_scheduler.cancelAll();
		bindControls();
	}

	@Override
	public void utilityInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand()));
	}
}