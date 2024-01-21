// Fill out your copyright notice in the Description page of Project Settings.


#include "Flock.h"
#include "Components/InstancedStaticMeshComponent.h"

DECLARE_STATS_GROUP(TEXT("BoidSimulation"), STATGROUP_BoidSimulation, STATCAT_Advanced);

DECLARE_CYCLE_STAT(TEXT("Simulate (GT)"), STAT_Simulate_GameThread, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Simulate (Task)"), STAT_Simulate_WorkerThread, STATGROUP_BoidSimulation);

namespace BoidSimulationCVars
{
static TAutoConsoleVariable<bool> UseMultithreading{
	TEXT("BoidSimulation.UseMultithreading"),
	false,
	TEXT("")};

static TAutoConsoleVariable<int32> BatchSize{
	TEXT("BoidSimulation.Multithreading.BatchSize"),\
	64,
	TEXT("")};
	
static TAutoConsoleVariable<bool> DrawDebugBoundsSphere{
	TEXT("BoidSimulation.DrawDebugBoundsSphere"),
	false,
	TEXT("")};

static TAutoConsoleVariable<float> CohesionStrength{
	TEXT("BoidSimulation.CohesionStrength"),
	0.75f,
	TEXT("")};

static TAutoConsoleVariable<float> AvoidanceStrength{
	TEXT("BoidSimulation.AvoidanceStrength"),
	0.75f,
	TEXT("")};

static TAutoConsoleVariable<float> AlignmentStrength{
	TEXT("BoidSimulation.AlignmentStrength"),
	0.75f,
	TEXT("")};
}

AFlock::AFlock(const FObjectInitializer& ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = true;
	
	Mesh = ObjectInitializer.CreateDefaultSubobject<UInstancedStaticMeshComponent>(this, TEXT("Mesh"));
	Mesh->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	SetRootComponent(Mesh);
}

void AFlock::BeginPlay()
{
	Super::BeginPlay();

	checkf(NumInstances > 0, TEXT("NumInstances == %i"), NumInstances);
	checkf(BoundsRadius > 0.f, TEXT("Radius == %f"), BoundsRadius);

	TArray<FTransform> Transforms;
	Transforms.Reserve(NumInstances);
	
	for (int32 i = 0; i < NumInstances; ++i)
	{
		const FVector RandomLocation = FMath::VRand() * FMath::RandRange(0.0, static_cast<double>(BoundsRadius));
		const FRotator RandomRotation = FRotator{FMath::RandRange(-180.0, 180.0), FMath::RandRange(-180.0, 180.0), 0.0};
		Transforms.Emplace(RandomRotation, RandomLocation);
	}

	Mesh->AddInstances(Transforms, false, false);
}

UE_NODISCARD FORCEINLINE FVector LerpNormals(const FVector& A, const FVector& B, const double Alpha)
{
	const FQuat RotationDifference = FQuat::FindBetweenNormals(A, B);

	FVector Axis; double Angle;
	RotationDifference.ToAxisAndAngle(Axis, Angle);

	return FQuat{Axis, Angle * Alpha}.RotateVector(A);
}

void AFlock::Cohere(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Cohere"), STAT_Cohere, STATGROUP_BoidSimulation);

	if (OtherRelevantBoidIndices.IsEmpty()) return;

	FVector AverageLocation = FVector::ZeroVector;
	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		FTransform OtherTransform{NoInit};
		Mesh->GetInstanceTransform(OtherBoidIndex, OtherTransform);

		AverageLocation += OtherTransform.GetTranslation();
	}
	AverageLocation /= OtherRelevantBoidIndices.Num();

	FTransform Transform{NoInit};
	Mesh->GetInstanceTransform(BoidIndex, Transform);

	const FVector DirToAverageLocation = (AverageLocation - Transform.GetTranslation()).GetSafeNormal();
	
	const double Alpha = FMath::GetMappedRangeValueClamped<double, double>({0.0, 15.0}, {0.0, BoidSimulationCVars::CohesionStrength.GetValueOnAnyThread()}, static_cast<double>(OtherRelevantBoidIndices.Num()));
	Directions[BoidIndex] = LerpNormals(Directions[BoidIndex], DirToAverageLocation, Alpha);
}

void AFlock::Avoid(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Avoid"), STAT_Avoid, STATGROUP_BoidSimulation);
	
	FTransform Transform{NoInit};
	Mesh->GetInstanceTransform(BoidIndex, Transform);

	FVector NewDirection = Directions[BoidIndex];

	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		FTransform OtherTransform{NoInit};
		Mesh->GetInstanceTransform(OtherBoidIndex, OtherTransform);

		const FVector Translation = FTransform::SubtractTranslations(Transform, OtherTransform);
		if (UNLIKELY(Translation.SizeSquared() < UE_DOUBLE_KINDA_SMALL_NUMBER)) continue;
		
		const double Dist = Translation.Size();

		NewDirection += Translation * (((1.0 - (Dist / BoidsSearchNearbyRadius)) / Dist) * BoidSimulationCVars::AvoidanceStrength.GetValueOnAnyThread());
	}

	NewDirection.Normalize();
	
	Directions[BoidIndex] = NewDirection;
}

void AFlock::Align(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Align"), STAT_Align, STATGROUP_BoidSimulation);

	if (OtherRelevantBoidIndices.IsEmpty()) return;

	FVector AverageDirection = FVector::ZeroVector;
	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		AverageDirection += Directions[OtherBoidIndex];
	}
	AverageDirection /= OtherRelevantBoidIndices.Num();
	AverageDirection.Normalize();

	const double Alpha = FMath::Min(1.0, static_cast<double>(OtherRelevantBoidIndices.Num()) / 15.0) * BoidSimulationCVars::AlignmentStrength.GetValueOnAnyThread();
	Directions[BoidIndex] = LerpNormals(Directions[BoidIndex], AverageDirection, Alpha);
}

void AFlock::Constrain(FVector& Direction, const int32 BoidIndex) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Constrain"), STAT_Constrain, STATGROUP_BoidSimulation);
	
	FTransform Transform{NoInit};
	Mesh->GetInstanceTransform(BoidIndex, Transform);
	
	// Confine to bounds
	if (Transform.GetTranslation().SizeSquared() > FMath::Square(BoundsRadius - BoidsSearchNearbyRadius - UE_DOUBLE_KINDA_SMALL_NUMBER))
	{
		const double DistFromOrigin = Transform.GetTranslation().Size();
		const FVector DirFromOrigin = Transform.GetTranslation() / DistFromOrigin;
		
		FVector RightAxis = Direction ^ DirFromOrigin;

		FVector TargetDirection;
		if (LIKELY(RightAxis.SizeSquared() > UE_DOUBLE_KINDA_SMALL_NUMBER))
		{
			RightAxis = RightAxis.GetUnsafeNormal();
			TargetDirection = RightAxis.RotateAngleAxisRad(UE_DOUBLE_PI / 2.0, DirFromOrigin).RotateAngleAxisRad(-UE_DOUBLE_PI / 4.0, RightAxis);
		}
		else
		{
			TargetDirection = -DirFromOrigin;
		}

		const double Alpha = FMath::GetMappedRangeValueUnclamped<double, double>({BoundsRadius - BoidsSearchNearbyRadius - UE_DOUBLE_KINDA_SMALL_NUMBER, BoundsRadius}, {0.0, 1.0}, DistFromOrigin);
		Direction = LerpNormals(Direction, TargetDirection, Alpha);
	}
}

void AFlock::SimulateSynchronously(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_Simulate_GameThread);

	TArray<FVector> Directions;
	Directions.Reserve(NumInstances);
	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform Transform{NoInit};
		verify(Mesh->GetInstanceTransform(i, Transform));
		Directions.Add(Transform.GetUnitAxis(EAxis::X));
	}

	TArray<int32, TInlineAllocator<32>> OtherRelevantBoids;// Declared out here to preserve slack
	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform Transform{NoInit};
		Mesh->GetInstanceTransform(i, Transform);

		{
			DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Find Nearby Boids"), STAT_FindNearbyBoids, STATGROUP_BoidSimulation);
			
			OtherRelevantBoids.Reset();
			for (int32 j = 0; j < NumInstances; ++j)
			{
				if (UNLIKELY(i == j)) continue;

				FTransform OtherTransform{NoInit};
				Mesh->GetInstanceTransform(j, OtherTransform);

				const FVector Translation = FTransform::SubtractTranslations(Transform, OtherTransform);

				// Not relevant if outside of search-radius or 
				if (Translation.SizeSquared() > FMath::Square(BoidsSearchNearbyRadius) ||
					(Directions[i] | Translation.GetSafeNormal()) <= -0.25) continue;
				
				OtherRelevantBoids.Add(j);
			}
		}

		Cohere(Directions, i, OtherRelevantBoids);
		Avoid(Directions, i, OtherRelevantBoids);
		Align(Directions, i, OtherRelevantBoids);
		Constrain(Directions[i], i);
	}

	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform Transform{NoInit};
		Mesh->GetInstanceTransform(i, Transform);

		Transform.AddToTranslation(Directions[i] * MovementSpeed * DeltaTime);
		Transform.SetRotation(Directions[i].ToOrientationQuat());
		Mesh->UpdateInstanceTransform(i, Transform);
	}
}

void AFlock::SimulateAsynchronously(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_Simulate_GameThread);
}


void AFlock::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidSimulationCVars::UseMultithreading.GetValueOnGameThread())
	{
		SimulateAsynchronously(DeltaTime);
	}
	else
	{
		SimulateSynchronously(DeltaTime);
	}

	Mesh->MarkRenderStateDirty();

#if UE_BUILD_DEVELOPMENT
	if (BoidSimulationCVars::DrawDebugBoundsSphere.GetValueOnGameThread())
	{
		DrawDebugSphere(GetWorld(), GetActorLocation(), BoundsRadius, 16, FColor::Blue);
	}
#endif
}

