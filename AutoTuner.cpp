#include "AutoTuner.h"
#include "Engine/Engine.h"

// ─────────────────────────────────────────────────────────────
// Update — runs every tick
// Returns PID output AND adjusts gains if tuning
// ─────────────────────────────────────────────────────────────
float FAutoTuner::Update(float Error, float DeltaTime)
{
	if (bTuning)
		DetectOscillation(Error, DeltaTime);

	// Standard PID compute
	float P = Kp * Error;

	Integral += Error * DeltaTime;
	Integral  = FMath::Clamp(Integral, -50.f, 50.f);
	float I   = Ki * Integral;

	float Derivative = (DeltaTime > 0.f)
		? (Error - PrevError) / DeltaTime : 0.f;
	PrevError = Error;
	float D   = Kd * Derivative;

	return P + I + D;
}

// ─────────────────────────────────────────────────────────────
// GetTestInput
// Returns alternating +/- test movement for axis testing
// This is what the auto-tuner commands to each axis
// ─────────────────────────────────────────────────────────────
float FAutoTuner::GetTestInput(float DeltaTime)
{
	if (!bTuning || bTuned) return 0.f;

	TestTimer += DeltaTime;

	// Alternate direction every TestDuration seconds
	if (TestTimer >= TestDuration)
	{
		TestTimer      = 0.f;
		bTestPositive  = !bTestPositive;
	}

	// Small test amplitude — enough to measure response
	// but not so large it crashes drone
	float Amplitude = 0.15f;
	TestInput = bTestPositive ? Amplitude : -Amplitude;

	return TestInput;
}

// ─────────────────────────────────────────────────────────────
// StartTuning
// ─────────────────────────────────────────────────────────────
void FAutoTuner::StartTuning()
{
	bTuning       = true;
	bTuned        = false;
	OscTimer      = 0.f;
	PeakCount     = 0;
	OscAmplitude  = 0.f;
	bRising       = true;
	TestTimer     = 0.f;
	bTestPositive = true;
	LastPeakTime  = 0.f;
	Tu            = 0.f;

	// Start with higher Kp to induce oscillation
	Kp = FMath::Max(Kp * 1.5f, 2.f);
	Ki = 0.f;
	Kd = 0.f;

	Reset();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 3.f,
			FColor::Yellow,
			FString::Printf(
				TEXT("[%s] TUNING STARTED"), *AxisName));
}

// ─────────────────────────────────────────────────────────────
// StopTuning
// ─────────────────────────────────────────────────────────────
void FAutoTuner::StopTuning()
{
	bTuning   = false;
	bTuned    = true;
	TestInput = 0.f;
	Reset();

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.f,
			FColor::Green,
			FString::Printf(
				TEXT("[%s] TUNED Kp=%.3f Ki=%.3f Kd=%.3f"),
				*AxisName, Kp, Ki, Kd));
}

// ─────────────────────────────────────────────────────────────
// DetectOscillation — Ziegler-Nichols method
// Watches error signal peaks to find Ku and Tu
// ─────────────────────────────────────────────────────────────
void FAutoTuner::DetectOscillation(
	float Error, float DeltaTime)
{
	OscTimer += DeltaTime;
	bool bCurrentRising = (Error > PrevError);

	// Detect direction change = peak found
	if (bCurrentRising != bRising)
	{
		float PeakValue = FMath::Abs(Error);
		OscAmplitude = FMath::Max(OscAmplitude, PeakValue);

		if (PeakCount > 0)
		{
			float Period = (OscTimer - LastPeakTime) * 2.f;
			if (PeakValue > 2.f && Period > 0.05f)
				Tu = Tu * 0.7f + Period * 0.3f;
		}

		LastPeakTime = OscTimer;
		PeakCount++;
		bRising = bCurrentRising;

		// Enough oscillations → apply Z-N formula
		if (PeakCount >= 6 && OscAmplitude > 2.f)
		{
			Ku = Kp;
			ApplyZieglerNichols();
			StopTuning();
			return;
		}
	}

	// Safety: oscillating too violently → reduce Kp
	if (OscAmplitude > 300.f)
	{
		Kp          *= 0.8f;
		OscAmplitude = 0.f;
	}

	// Timeout: no oscillation → increase Kp to induce it
	if (OscTimer > 8.f && PeakCount < 3)
	{
		Kp        = FMath::Clamp(Kp * 1.3f, MinKp, MaxKp);
		OscTimer  = 0.f;
		PeakCount = 0;
	}
}

// ─────────────────────────────────────────────────────────────
// ApplyZieglerNichols
// Classic Z-N PID formula
// ─────────────────────────────────────────────────────────────
void FAutoTuner::ApplyZieglerNichols()
{
	if (Tu < 0.01f) Tu = 0.5f;

	Kp = FMath::Clamp(0.6f  * Ku,        MinKp, MaxKp);
	Ki = FMath::Clamp(2.f   * Kp / Tu,   0.f,   MaxKi);
	Kd = FMath::Clamp(Kp    * Tu / 8.f,  0.f,   MaxKd);

	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 8.f,
			FColor::Cyan,
			FString::Printf(
				TEXT("[%s] Z-N: Ku=%.2f Tu=%.2f"
					" Kp=%.3f Ki=%.3f Kd=%.3f"),
				*AxisName, Ku, Tu, Kp, Ki, Kd));
}
