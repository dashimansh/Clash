#pragma once
#include "CoreMinimal.h"
#include "AutoTuner.h"

// ─────────────────────────────────────────────────────────────
// Sensor data filled every tick from DronePawn
// ─────────────────────────────────────────────────────────────
struct FSensorData
{
	float   ActualRoll     = 0.f;
	float   ActualPitch    = 0.f;
	float   ActualYaw      = 0.f;
	float   ActualAltitude = 0.f;
	float   ActualVelZ     = 0.f;
	FVector ActualPosition  = FVector::ZeroVector;
	FVector ActualVelocity  = FVector::ZeroVector;
	FVector WindDisturbance = FVector::ZeroVector;
};

// ─────────────────────────────────────────────────────────────
// Motor output — normalized 0.0 to 1.0
//
//     FL(CW)    FR(CCW)
//        \      /
//         ------
//        /      \
//     BL(CCW)   BR(CW)
// ─────────────────────────────────────────────────────────────
struct FMotorOutput
{
	float FL = 0.5f;
	float FR = 0.5f;
	float BL = 0.5f;
	float BR = 0.5f;
};

// ─────────────────────────────────────────────────────────────
// Auto-tune axis sequence
// ─────────────────────────────────────────────────────────────
enum class ETuneAxis : uint8
{
	Roll  = 0,
	Pitch = 1,
	Yaw   = 2,
	Alt   = 3,
	Done  = 4
};

// ─────────────────────────────────────────────────────────────
// Flight Controller
// Brain of drone — PID per axis → correct motor outputs
// ─────────────────────────────────────────────────────────────
struct FFlightController
{
	// ── Target setpoints ──
	float TargetRoll     = 0.f;
	float TargetPitch    = 0.f;
	float TargetYaw      = 0.f;
	float TargetAltitude = 0.f;
	float TargetThrottle = 0.5f;

	// ── PID Gains ──
	float Roll_Kp  = 8.f;   float Roll_Ki  = 0.05f; float Roll_Kd  = 3.f;
	float Pitch_Kp = 8.f;   float Pitch_Ki = 0.05f; float Pitch_Kd = 3.f;
	float Yaw_Kp   = 5.f;   float Yaw_Ki   = 0.02f; float Yaw_Kd   = 2.f;
	float Alt_Kp   = 15.f;  float Alt_Ki   = 0.05f; float Alt_Kd   = 5.f;

	// ── Auto Tuners ──
	FAutoTuner Tuner_Roll;
	FAutoTuner Tuner_Pitch;
	FAutoTuner Tuner_Yaw;
	FAutoTuner Tuner_Alt;

	// ── Auto-tune state ──
	bool       bAutoTuning   = false;
	bool       bTuneComplete = false;
	FString    TuneStatus    = TEXT("IDLE");
	ETuneAxis  CurrentAxis   = ETuneAxis::Roll;

	// ── PID outputs (for HUD + debug) ──
	float Out_Roll  = 0.f;
	float Out_Pitch = 0.f;
	float Out_Yaw   = 0.f;
	float Out_Alt   = 0.f;

	// ── Initialize ──
	void Init()
	{
		Tuner_Roll.Init (TEXT("ROLL"),  Roll_Kp,  Roll_Ki,  Roll_Kd);
		Tuner_Pitch.Init(TEXT("PITCH"), Pitch_Kp, Pitch_Ki, Pitch_Kd);
		Tuner_Yaw.Init  (TEXT("YAW"),   Yaw_Kp,   Yaw_Ki,   Yaw_Kd);
		Tuner_Alt.Init  (TEXT("ALT"),   Alt_Kp,   Alt_Ki,   Alt_Kd);
	}

	// ── Main update every tick ──
	FMotorOutput Update(const FSensorData& S, float DeltaTime);

	// ── Auto tune ──
	void StartAutoTune();
	void StopAutoTune();

	// ── Reset integrators ──
	void Reset()
	{
		Tuner_Roll.Reset();
		Tuner_Pitch.Reset();
		Tuner_Yaw.Reset();
		Tuner_Alt.Reset();
	}

private:
	struct FPIDState
	{
		float Integral  = 0.f;
		float PrevError = 0.f;
		void  Reset() { Integral = 0.f; PrevError = 0.f; }
	};

	FPIDState PID_Roll;
	FPIDState PID_Pitch;
	FPIDState PID_Yaw;
	FPIDState PID_Alt;

	float ComputePID(FPIDState& S, float Kp, float Ki,
		float Kd, float Error, float Dt);

	void  SyncGainsFromTuners();
	void  UpdateTuneSequence(
		const FSensorData& S, float DeltaTime);
	void  AdvanceToNextAxis();
};
