package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BackBrace;
import org.firstinspires.ftc.teamcode.subsystems.FrontBrace;
import org.firstinspires.ftc.teamcode.subsystems.Guide;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.Tensioner;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.OpModeWrapper;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="Right Medium 5 Auto", group="")
public class RightMedium5Auto extends OpModeWrapper {
	private SampleMecanumDrive mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	private Webcam webcam;
	private Guide guide;
	private Tensioner tensioner;
	private BackBrace backBrace;
	private FrontBrace frontBrace;
	TrajectorySequenceStorage sequence;
	
	@Override
	public void superInit() {
		mecanum = new SampleMecanumDrive(hardwareMap);
		intake = new Intake(Intake.IntakePos.OPEN);
		arm = new Arm();
		wrist = new Wrist();
		limitSwitch = new LimitSwitch();
		lift = new Lift();
		webcam = new Webcam();
		guide = new Guide();
		backBrace = new BackBrace();
		frontBrace = new FrontBrace();
		tensioner = new Tensioner(Tensioner.RunMode.DEPLOY);
		sequence = new TrajectorySequenceStorage().rightMedium5(
										r,
										mecanum,
				intake,
				arm,
				lift,
				wrist,
				guide,
				webcam,
				backBrace,
				frontBrace
								);
		
	}
	
	@Override
	public void registerTriggers() {
	
	}
	
	@Override
	public void superInit_Loop() {
		webcam.readStream();
	}
	
	@Override
	public void superStart() {
		sequence.addRightParkSequence(webcam.getTagId());
		webcam.closeStream();
		sequence.startFollowSetSequenceAsync();
	}
	
	@Override
	public void superLoop() {
		sequence.followSetSequenceAsync();
		lift.limitSwitchInput(limitSwitch.limitSwitchEX.buttonState());
	}
	
	@Override
	public void superStop() {
	
	}
}
