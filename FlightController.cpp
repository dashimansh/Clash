#include "FlightController.h"
#include "Engine/Engine.h"

// ─────────────────────────────────────────────────────────────
// Main Update — runs every Tick
// ─────────────────────────────────────────────────────────────
FMotorOutput FFlightController::Update(
	const FSensorData& S, float DeltaTime)
{
	// ── Step 1: Calculate errors ──
	float RollError  = TargetRoll     - S.ActualRoll;
	float PitchError = TargetPitch    - S.ActualPitch;
	float AltError   = TargetAltitude - S.ActualAltitude;

	// Normalize yaw error to -180/+180
	float YawError = TargetYaw - S.ActualYaw;
	while (YawError >  180.f) YawError -= 360.f;
	while (YawError < -180.f) YawError += 360.f;

	// ── Step 2: Run auto-tune sequence if active ──
	if (bAutoTuning)
		UpdateTuneSequence(S, DeltaTime);

	// ── Step 3: PID compute per axis ──
	if (bAutoTuning)
	{
		Out_Roll  = Tuner_Roll.Update (RollError,  DeltaTime);
		Out_Pitch = Tuner_Pitch.Update(PitchError, DeltaTime);
		Out_Yaw   = Tuner_Yaw.Update  (YawError,   DeltaTime);
		Out_Alt   = Tuner_Alt.Update  (AltError,   DeltaTime);
		SyncGainsFromTuners();
	}
	else
	{
		Out_Roll  = ComputePID(PID_Roll,  Roll_Kp,  Roll_Ki,
			Roll_Kd,  RollError,  DeltaTime);
		Out_Pitch = ComputePID(PID_Pitch, Pitch_Kp, Pitch_Ki,
			Pitch_Kd, PitchError, DeltaTime);
		Out_Yaw   = ComputePID(PID_Yaw,   Yaw_Kp,   Yaw_Ki,
			Yaw_Kd,   YawError,   DeltaTime);
		Out_Alt   = ComputePID(PID_Alt,   Alt_Kp,   Alt_Ki,
			Alt_Kd,   AltError,   DeltaTime);
	}

	// ── Step 4: Clamp outputs ──
	Out_Roll  = FMath::Clamp(Out_Roll,  -1.f, 1.f);
	Out_Pitch = FMath::Clamp(Out_Pitch, -1.f, 1.f);
	Out_Yaw   = FMath::Clamp(Out_Yaw,  -1.f, 1.f);
	Out_Alt   = FMath::Clamp(Out_Alt,  -1.f, 1.f);

	// ── Step 5: CORRECT MOTOR MIXER ──
	//
	//     FL(CW)    FR(CCW)
	//        \      /
	//         ------
	//        /      \
	//     BL(CCW)   BR(CW)
	//
	// PITCH FORWARD (+) → FL↓ FR↓  BL↑ BR↑
	// PITCH BACKWARD(-) → FL↑ FR↑  BL↓ BR↓
	// ROLL RIGHT    (+) → FL↑ BL↑  FR↓ BR↓
	// ROLL LEFT     (-) → FL↓ BL↓  FR↑ BR↑
	// YAW RIGHT     (+) → FL↑ BR↑  FR↓ BL↓ (torque)
	// YAW LEFT      (-) → FL↓ BR↓  FR↑ BL↑ (torque)

	float T = FMath::Clamp(TargetThrottle, 0.1f, 1.f);

	FMotorOutput M;
	M.FL = FMath::Clamp(T - Out_Pitch + Out_Roll - Out_Yaw, 0.f, 1.f);
	M.FR = FMath::Clamp(T - Out_Pitch - Out_Roll + Out_Yaw, 0.f, 1.f);
	M.BL = FMath::Clamp(T + Out_Pitch + Out_Roll + Out_Yaw, 0.f, 1.f);
	M.BR = FMath::Clamp(T + Out_Pitch - Out_Roll - Out_Yaw, 0.f, 1.f);

	// ── Step 6: Debug display ──
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(80, 0.f,
			FColor::Green,
			FString::Printf(
				TEXT("FC | R:%.2f P:%.2f Y:%.2f A:%.2f"),
				Out_Roll, Out_Pitch, Out_Yaw, Out_Alt));

		GEngine->AddOnScreenDebugMessage(81, 0.f,
			FColor::Cyan,
			FString::Printf(
				TEXT("MOTORS | FL:%.2f FR:%.2f BL:%.2f BR:%.2f"),
				M.FL, M.FR, M.BL, M.BR));

		GEngine->AddOnScreenDebugMessage(82, 0.f,
			FColor::Yellow,
			FString::Printf(
				TEXT("GAINS | Rp:%.2f Pp:%.2f Yp:%.2f Ap:%.2f"),
				Roll_Kp, Pitch_Kp, Yaw_Kp, Alt_Kp));

		if (bAutoTuning)
			GEngine->AddOnScreenDebugMessage(83, 0.f,
				FColor::Orange,
				FString::Printf(
					TEXT("AUTO-TUNE: %s"), *TuneStatus));
	}

	return M;
}

// ─────────────────────────────────────────────────────────────
// Standard PID compute
// ─────────────────────────────────────────────────────────────
float FFlightController::ComputePID(
	FPIDState& S, float Kp, float Ki, float Kd,
	float Error, float Dt)
{
	float P        = Kp * Error;
	S.Integral    += Error * Dt;
	S.Integral     = FMath::Clamp(S.Integral, -50.f, 50.f);
	float I        = Ki * S.Integral;
	float Deriv    = (Dt > 0.f)
		? (Error - S.PrevError) / Dt : 0.f;
	S.PrevError    = Error;
	float D        = Kd * Deriv;
	return P + I + D;
}

// ─────────────────────────────────────────────────────────────
// UpdateTuneSequence
// Tests one axis at a time with controlled inputs
// Each axis gets correct motor differential
// ─────────────────────────────────────────────────────────────
void FFlightController::UpdateTuneSequence(
	const FSensorData& S, float DeltaTime)
{
	switch (CurrentAxis)
	{
	case ETuneAxis::Roll:
	{
		// Test roll → FL+BL vs FR+BR
		float TestInput = Tuner_Roll.GetTestInput(DeltaTime);
		TargetRoll = TestInput * 15.f; // 15° test roll
		TuneStatus = TEXT("TUNING ROLL...");

		if (Tuner_Roll.bTuned)
		{
			TargetRoll = 0.f;
			AdvanceToNextAxis();
		}
		break;
	}
	case ETuneAxis::Pitch:
	{
		// Test pitch → FL+FR vs BL+BR
		float TestInput = Tuner_Pitch.GetTestInput(DeltaTime);
		TargetPitch = TestInput * 15.f; // 15° test pitch
		TuneStatus  = TEXT("TUNING PITCH...");

		if (Tuner_Pitch.bTuned)
		{
			TargetPitch = 0.f;
			AdvanceToNextAxis();
		}
		break;
	}
	case ETuneAxis::Yaw:
	{
		// Test yaw → CW vs CCW motors
		float TestInput = Tuner_Yaw.GetTestInput(DeltaTime);
		TargetYaw  = S.ActualYaw + TestInput * 20.f;
		TuneStatus = TEXT("TUNING YAW...");

		if (Tuner_Yaw.bTuned)
		{
			TargetYaw = S.ActualYaw;
			AdvanceToNextAxis();
		}
		break;
	}
	case ETuneAxis::Alt:
	{
		// Test altitude → all motors equal adjustment
		float TestInput = Tuner_Alt.GetTestInput(DeltaTime);
		TargetAltitude = S.ActualAltitude
			+ TestInput * 200.f; // 2m test movement
		TuneStatus = TEXT("TUNING ALT...");

		if (Tuner_Alt.bTuned)
		{
			TargetAltitude = S.ActualAltitude;
			AdvanceToNextAxis();
		}
		break;
	}
	case ETuneAxis::Done:
	{
		TuneStatus    = TEXT("COMPLETE");
		bTuneComplete = true;
		bAutoTuning   = false;

		if (GEngine)
			GEngine->AddOnScreenDebugMessage(-1, 8.f,
				FColor::Green,
				TEXT(">> ALL AXES TUNED"
					" — GAINS READY FOR REAL DRONE <<"));
		break;
	}
	}
}

// ─────────────────────────────────────────────────────────────
// AdvanceToNextAxis
// ─────────────────────────────────────────────────────────────
void FFlightController::AdvanceToNextAxis()
{
	switch (CurrentAxis)
	{
	case ETuneAxis::Roll:
		CurrentAxis = ETuneAxis::Pitch;
		if (GEngine) GEngine->AddOnScreenDebugMessage(
			-1, 3.f, FColor::Cyan,
			TEXT("Roll tuned → Tuning Pitch..."));
		break;
	case ETuneAxis::Pitch:
		CurrentAxis = ETuneAxis::Yaw;
		if (GEngine) GEngine->AddOnScreenDebugMessage(
			-1, 3.f, FColor::Cyan,
			TEXT("Pitch tuned → Tuning Yaw..."));
		break;
	case ETuneAxis::Yaw:
		CurrentAxis = ETuneAxis::Alt;
		if (GEngine) GEngine->AddOnScreenDebugMessage(
			-1, 3.f, FColor::Cyan,
			TEXT("Yaw tuned → Tuning Alt..."));
		break;
	case ETuneAxis::Alt:
		CurrentAxis = ETuneAxis::Done;
		break;
	default:
		break;
	}
}

// ─────────────────────────────────────────────────────────────
// Start Auto Tune
// ─────────────────────────────────────────────────────────────
void FFlightController::StartAutoTune()
{
	bAutoTuning   = true;
	bTuneComplete = false;
	CurrentAxis   = ETuneAxis::Roll;
	TuneStatus    = TEXT("TUNING ROLL...");

	Tuner_Roll.StartTuning();
	Tuner_Pitch.StartTuning();
	Tuner_Yaw.StartTuning();
	Tuner_Alt.StartTuning();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f,
			FColor::Yellow,
			TEXT(">> AUTO-TUNE STARTED <<"));
}

// ─────────────────────────────────────────────────────────────
// Stop Auto Tune
// ─────────────────────────────────────────────────────────────
void FFlightController::StopAutoTune()
{
	bAutoTuning = false;
	TuneStatus  = TEXT("STOPPED");
	TargetRoll  = 0.f;
	TargetPitch = 0.f;

	Tuner_Roll.StopTuning();
	Tuner_Pitch.StopTuning();
	Tuner_Yaw.StopTuning();
	Tuner_Alt.StopTuning();

	SyncGainsFromTuners();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f,
			FColor::Green,
			FString::Printf(
				TEXT("TUNED | R:%.2f P:%.2f Y:%.2f A:%.2f"),
				Roll_Kp, Pitch_Kp, Yaw_Kp, Alt_Kp));
}

// ─────────────────────────────────────────────────────────────
// Sync tuned gains back to exposed variables
// ─────────────────────────────────────────────────────────────
void FFlightController::SyncGainsFromTuners()
{
	Roll_Kp  = Tuner_Roll.Kp;
	Roll_Ki  = Tuner_Roll.Ki;
	Roll_Kd  = Tuner_Roll.Kd;

	Pitch_Kp = Tuner_Pitch.Kp;
	Pitch_Ki = Tuner_Pitch.Ki;
	Pitch_Kd = Tuner_Pitch.Kd;

	Yaw_Kp   = Tuner_Yaw.Kp;
	Yaw_Ki   = Tuner_Yaw.Ki;
	Yaw_Kd   = Tuner_Yaw.Kd;

	Alt_Kp   = Tuner_Alt.Kp;
	Alt_Ki   = Tuner_Alt.Ki;
	Alt_Kd   = Tuner_Alt.Kd;
}
