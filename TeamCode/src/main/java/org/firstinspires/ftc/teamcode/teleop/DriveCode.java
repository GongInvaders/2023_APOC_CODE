package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BackBrace;
import org.firstinspires.ftc.teamcode.subsystems.FrontBrace;
import org.firstinspires.ftc.teamcode.subsystems.Guide;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.OpModeWrapper;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

import com.qualcomm.robotcore.hardware.IMU;


// brodie mode biggest bird edition
@TeleOp(name = "brodie mode \nbiggest bird edition \nft. CRACKER BARREL \nft. blue lobster \n\n\nBIIIIIIIG", group = "!up to the ftrony!")
public class DriveCode extends OpModeWrapper {
	private MecanumDriveBase mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	private BackBrace backBrace;
	private FrontBrace frontBrace;
	private Guide guide;
	private IMU imu;
	public static double initialPitch = 0;

	double checkTime;
	boolean throttle;
	public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
			RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD; //was backward
	public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
			RevHubOrientationOnRobot.UsbFacingDirection.UP;




	@Override
	public void superInit() {
		imu = hardwareMap.get(IMU.class, "imu");

		mecanum = new MecanumDriveBase(true, MecanumDriveBase.DriveModes.CENTRIC);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake(Intake.IntakePos.CLOSED);
		limitSwitch = new LimitSwitch();
		lift = new Lift();
		guide = new Guide();
		backBrace = new BackBrace();
		frontBrace = new FrontBrace();

		//initialPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
		IMU.Parameters myIMUParameters;

		myIMUParameters = new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
						RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
		);
		imu.initialize(myIMUParameters);


	}

	@Override
	public void registerTriggers() {
		gamepadEX2.leftY.applyDeadZone(0.5);
		
		gamepadEX2.right_trigger
				.thresholdTrigger(0.1)
				.onTrue(() -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					//checkTime = RobotConfig.elapsedTime.time();
				})
				.onFalse(() -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				});
		gamepadEX2.left_trigger
				.thresholdTrigger(0.1)
				.onTrue(() -> {
					r.setDelivery(false);
					r.setPickup(true);
				})
				.onFalse(() -> {
					r.setPickup(false);
				});
		
		gamepadEX2.back
				.onPress()
				.onTrue(() -> {
					lift.resetEncoders();
				});
		
		gamepadEX1.back
				.onPress()
				.onTrue(() -> {
					mecanum.flipDriveBase();
				});
		gamepadEX1.dpad_down
				.onPress()
				.onTrue(() -> {
					backBrace.presetbracePosition(BackBrace.BracePositions.DOWN);
					frontBrace.presetbracePosition(FrontBrace.BracePositions.DOWN);
				});
		gamepadEX1.dpad_up
				.onPress()
				.onTrue(() -> {
					backBrace.presetbracePosition(BackBrace.BracePositions.UP);
					frontBrace.presetbracePosition(FrontBrace.BracePositions.UP);
				});

		gamepadEX2.left_bumper
				.isPressed()
				.onTrue(() -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.onFalse(() -> {
					guide.presetTargetPosition(Guide.GuidePos.STOW);

				});
		
		gamepadEX1.left_bumper
				.isPressed()
				.onTrue(() -> {
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				});

		gamepadEX1.right_bumper
				.isPressed()
				.onTrue(() -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				});
		
		gamepadEX1.right_stick_button
				.onPress()
				.toggleOnTrue(() -> throttle = true)
				.toggleOnFalse(() -> throttle = false);
	}

	@Override
	public void superInit_Loop() {
	}

	@Override
	public void superStart() {
	
	}

	@Override
	public void superLoop() {
		YawPitchRollAngles robotOrientation;
		robotOrientation = imu.getRobotYawPitchRollAngles();

		mecanum.tankDrive(
				gamepadEX1.leftY.getValue(),
				gamepadEX1.rightY.getValue(),
				gamepadEX1.left_trigger.getValue(),
				gamepadEX1.right_trigger.getValue(),
				throttle
		);
		
		lift.liftInputs(
				gamepadEX2.leftY.getValue(),
				limitSwitch.limitSwitchEX.buttonState(),
				gamepadEX2.y.buttonState(),
				gamepadEX2.b.buttonState(),
				gamepadEX2.x.buttonState(),
				gamepadEX2.a.buttonState()
		);
		if(lift.getLiftPos()<-500) {
			if (robotOrientation.getPitch(AngleUnit.DEGREES) > 6) {
				backBrace.presetbracePosition(BackBrace.BracePositions.DOWN);
				frontBrace.presetbracePosition(FrontBrace.BracePositions.DOWN);
				/*if (robotOrientation.getPitch(AngleUnit.DEGREES)>20){
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				}*/
			} else if (robotOrientation.getPitch(AngleUnit.DEGREES) < 0) {
				backBrace.presetbracePosition(BackBrace.BracePositions.DOWN);
				frontBrace.presetbracePosition(FrontBrace.BracePositions.DOWN);
				/*if (robotOrientation.getPitch(AngleUnit.DEGREES)>20){
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				}*/
			} else if (Math.abs(gamepad1.left_stick_y + gamepad1.right_stick_y) > 1.5) {
				backBrace.presetbracePosition(BackBrace.BracePositions.DOWN);
				frontBrace.presetbracePosition(FrontBrace.BracePositions.DOWN);
			} else {
				backBrace.presetbracePosition(BackBrace.BracePositions.UP);
				frontBrace.presetbracePosition(FrontBrace.BracePositions.UP);
			}

		} else if (gamepad2.right_bumper) {
			backBrace.presetbracePosition(BackBrace.BracePositions.FLIP);
			frontBrace.presetbracePosition(FrontBrace.BracePositions.FLIP);

		} else {
			backBrace.presetbracePosition(BackBrace.BracePositions.UP);
			frontBrace.presetbracePosition(FrontBrace.BracePositions.UP);
		}
		telemetry.addData("Pitch", robotOrientation.getPitch(AngleUnit.DEGREES));
		telemetry.addData("Back Brace", backBrace.getEncoder());
		telemetry.addData("Front Brace", frontBrace.getEncoder());
		telemetry.addData("Lift Height: ", lift.getLiftPos());


	}

	@Override
	public void superStop() {

	}
}
