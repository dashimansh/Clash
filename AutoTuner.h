#pragma once
#include "CoreMinimal.h"

// ─────────────────────────────────────────────────────────────
// FAutoTuner — Plain C++ struct
// Ziegler-Nichols PID auto-tuner
// Tests each axis independently with controlled inputs
// ─────────────────────────────────────────────────────────────

struct FAutoTuner
{
	// ── Current PID gains (auto-adjusted) ──
	float Kp = 1.f;
	float Ki = 0.f;
	float Kd = 0.1f;

	// ── Limits ──
	float MaxKp = 50.f;
	float MaxKi = 5.f;
	float MaxKd = 20.f;
	float MinKp = 0.1f;

	// ── State ──
	bool    bTuning   = false;
	bool    bTuned    = false;
	FString AxisName  = TEXT("AXIS");

	// ── Ziegler-Nichols internals ──
	float Ku           = 0.f;   // Ultimate gain
	float Tu           = 0.f;   // Ultimate period
	float OscTimer     = 0.f;
	float LastPeakTime = 0.f;
	float OscAmplitude = 0.f;
	int32 PeakCount    = 0;
	bool  bRising      = true;

	// ── Test input for axis testing ──
	float TestInput      = 0.f;   // commanded test movement
	float TestTimer      = 0.f;
	float TestDuration   = 2.f;   // seconds per test step
	bool  bTestPositive  = true;  // alternates +/-

	// ── PID internal state ──
	float Integral  = 0.f;
	float PrevError = 0.f;

	// ── Initialize ──
	void Init(FString Name, float InitKp,
		float InitKi, float InitKd)
	{
		AxisName = Name;
		Kp = InitKp;
		Ki = InitKi;
		Kd = InitKd;
		Reset();
	}

	// ── Main update — returns PID output ──
	float Update(float Error, float DeltaTime);

	// ── Get test input to command on this axis ──
	// Returns a small test movement value (-1 to 1)
	float GetTestInput(float DeltaTime);

	// ── Control ──
	void StartTuning();
	void StopTuning();
	void Reset()
	{
		Integral  = 0.f;
		PrevError = 0.f;
	}

private:
	void DetectOscillation(float Error, float DeltaTime);
	void ApplyZieglerNichols();
};
