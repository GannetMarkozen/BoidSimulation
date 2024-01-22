// Fill out your copyright notice in the Description page of Project Settings.


#include "Flock.h"
#include "Components/InstancedStaticMeshComponent.h"

DECLARE_STATS_GROUP(TEXT("BoidSimulation"), STATGROUP_BoidSimulation, STATCAT_Advanced);

DECLARE_CYCLE_STAT(TEXT("Simulate (GT)"), STAT_Simulate_GameThread, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Simulate (Task)"), STAT_Simulate_WorkerThread, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Initialize Directions"), STAT_InitializeDirections, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Find Nearby Boids"), STAT_FindNearbyBoids, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Relocate Boid Cells"), STAT_RelocateBoidCells, STATGROUP_BoidSimulation);
DECLARE_CYCLE_STAT(TEXT("Relocate Boid Cells Blocking Time"), STAT_RelocateBoidCellsBlockingTime, STATGROUP_BoidSimulation);

namespace BoidSimulationCVars
{
static TAutoConsoleVariable<bool> EnableMultithreading{
	TEXT("BoidSimulation.EnableMultithreading"),
	true,
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
	Mesh->SetCanEverAffectNavigation(false);
	SetRootComponent(Mesh);
}

void AFlock::BeginPlay()
{
	Super::BeginPlay();

	checkf(NumInstances > 0, TEXT("NumInstances == %i"), NumInstances);
	checkf(BoundsRadius > 0.f, TEXT("Radius == %f"), BoundsRadius);

	const int32 NumCells = GetNumCells();
	BoidCells.SetNum(NumCells);
	BoidCellSpinLocks.SetNum(NumCells);

	TArray<FTransform> Transforms;
	Transforms.Reserve(NumInstances);
	
	for (int32 i = 0; i < NumInstances; ++i)
	{
		const FVector RandomLocation = FMath::VRand() * FMath::RandRange(0.0, static_cast<double>(BoundsRadius));
		const FRotator RandomRotation = FRotator{FMath::RandRange(-180.0, 180.0), FMath::RandRange(-180.0, 180.0), 0.0};
		Transforms.Emplace(RandomRotation, RandomLocation);

		BoidCells[GetCellIndex(RandomLocation)].Add(i);
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

void AFlock::Cohere(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
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
	
	const FVector DirToAverageLocation = (AverageLocation - Locations[BoidIndex]).GetSafeNormal();
	
	const double Alpha = FMath::GetMappedRangeValueClamped<double, double>({0.0, 15.0}, {0.0, BoidSimulationCVars::CohesionStrength.GetValueOnAnyThread()}, static_cast<double>(OtherRelevantBoidIndices.Num()));
	OutDirection = LerpNormals(OutDirection, DirToAverageLocation, Alpha);
}

void AFlock::RelocateBoidCell(const int32 BoidIndex, const FVector& LastLocation, const FVector& NewLocation)
{
	SCOPE_CYCLE_COUNTER(STAT_RelocateBoidCells);
	
	const int32 LastCell = GetCellIndex(LastLocation);
	const int32 NewCell = GetCellIndex(NewLocation);
	if (LastCell == NewCell) return;

	verify(BoidCells[LastCell].RemoveSingle(BoidIndex) != INDEX_NONE);
	check(!BoidCells[NewCell].Contains(BoidIndex));
	BoidCells[NewCell].Add(BoidIndex);
}

void AFlock::Avoid(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Avoid"), STAT_Avoid, STATGROUP_BoidSimulation);

	FVector NewDirection = OutDirection;

	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		const FVector Translation = Locations[BoidIndex] - Locations[OtherBoidIndex];
		if (UNLIKELY(Translation.SizeSquared() < UE_DOUBLE_KINDA_SMALL_NUMBER)) continue;
		
		const double Dist = Translation.Size();

		NewDirection += Translation * (((1.0 - (Dist / BoidsSearchNearbyRadius)) / Dist) * BoidSimulationCVars::AvoidanceStrength.GetValueOnAnyThread());
	}

	NewDirection.Normalize();
	
	OutDirection = NewDirection;
}

void AFlock::Align(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
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
	OutDirection = LerpNormals(OutDirection, AverageDirection, Alpha);
}

void AFlock::Constrain(FVector& RESTRICT OutDirection, const FVector& RESTRICT Location, const int32 BoidIndex) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Constrain"), STAT_Constrain, STATGROUP_BoidSimulation);
	
	// Confine to bounds
	if (Location.SizeSquared() > FMath::Square(BoundsRadius - BoidsSearchNearbyRadius - UE_DOUBLE_KINDA_SMALL_NUMBER))
	{
		const double DistFromOrigin = Location.Size();
		const FVector DirFromOrigin = Location / DistFromOrigin;
		
		FVector RightAxis = OutDirection ^ DirFromOrigin;
		
		FVector TargetDirection;
		if (LIKELY(RightAxis.SizeSquared() > UE_DOUBLE_KINDA_SMALL_NUMBER))
		{
			RightAxis = RightAxis.GetUnsafeNormal();
			TargetDirection = RightAxis.RotateAngleAxisRad(UE_DOUBLE_PI / 2.0, DirFromOrigin).RotateAngleAxisRad(-UE_DOUBLE_PI / 2.0, RightAxis);
		}
		else
		{
			TargetDirection = -DirFromOrigin;
		}

		const double Alpha = FMath::GetMappedRangeValueUnclamped<double, double>({BoundsRadius - BoidsSearchNearbyRadius - UE_DOUBLE_KINDA_SMALL_NUMBER, BoundsRadius}, {0.0, 1.0}, DistFromOrigin);
		OutDirection = LerpNormals(OutDirection, TargetDirection, Alpha);
	}
}

void AFlock::SimulateSynchronously(float DeltaTime)
{
#if 0
	SCOPE_CYCLE_COUNTER(STAT_Simulate_GameThread);
	
	TArray<FVector> Directions;
	{
		SCOPE_CYCLE_COUNTER(STAT_InitializeDirections);
		
		Directions.Reserve(NumInstances);
		for (int32 i = 0; i < NumInstances; ++i)
		{
			FTransform Transform{NoInit};
			verify(Mesh->GetInstanceTransform(i, Transform));
			Directions.Add(Transform.GetUnitAxis(EAxis::X));
		}
	}

	TArray<int32, TInlineAllocator<32>> OtherRelevantBoids;// Declared out here to preserve slack
	for (int32 i = 0; i < NumInstances; ++i)
	{
		FVector NewDirection = Directions[i];// Working with a copy rather than a reference to avoid false-sharing.
		{
			SCOPE_CYCLE_COUNTER(STAT_FindNearbyBoids);

			FTransform Transform{NoInit};
			Mesh->GetInstanceTransform(i, Transform);
			
			OtherRelevantBoids.Reset();
#if 0
			DrawDebugSphere(GetWorld(), GetActorTransform().TransformPosition(Transform.GetTranslation()), BoidsSearchNearbyRadius, 8, FColor::Green);
			
			const FVector Location = Transform.GetTranslation();

			const int32 CellDimensions = GetCellDimensions();
			
			const int32 StartX = FMath::Clamp(FMath::RoundToInt32((Location.X - BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0, CellDimensions - 1);
			const int32 EndX = FMath::Clamp(FMath::RoundToInt32((Location.X + BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0, CellDimensions - 1);

			const int32 StartY = FMath::Clamp(FMath::RoundToInt32((Location.Y - BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0, CellDimensions - 1);
			const int32 EndY = FMath::Clamp(FMath::RoundToInt32((Location.Y + BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0.0, CellDimensions - 1);

			const int32 StartZ = FMath::Clamp(FMath::RoundToInt32((Location.Z - BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0, CellDimensions - 1);
			const int32 EndZ = FMath::Clamp(FMath::RoundToInt32((Location.Z + BoidsSearchNearbyRadius + BoundsRadius) / CELL_SIZE), 0, CellDimensions - 1);

			for (int32 Z = StartZ; Z < EndZ + 1; ++Z)
			{
				for (int32 Y = StartY; Y < EndY + 1; ++Y)
				{
					for (int32 X = StartX; X < EndX + 1; ++X)
					{
						const FVector CellLocation = GetCellLocation(FIntVector{X, Y, Z});
						const bool bIntersects = FMath::SphereAABBIntersection(Transform.GetLocation(), FMath::Square(static_cast<double>(BoidsSearchNearbyRadius)), FBox{CellLocation - FVector{CELL_SIZE / 2.0}, CellLocation + FVector{CELL_SIZE / 2.0}});
						
						DrawDebugBox(GetWorld(), GetActorTransform().TransformPosition(GetCellLocation(FIntVector{X, Y, Z})), FVector{CELL_SIZE / 2.0}, bIntersects ? FColor::Cyan : FColor::Red);
					}
				}
			}
#endif

			ForEachNearbyBoid(Transform.GetLocation(), [&](const int32 BoidIndex, const FTransform& OtherTransform) -> void
			{
				const FVector Translation = FTransform::SubtractTranslations(Transform, OtherTransform);
				if ((NewDirection | Translation) <= -0.25) return;
				
				OtherRelevantBoids.Add(BoidIndex);
			});
		}
		
		Cohere(NewDirection, Directions, i, OtherRelevantBoids);
		Avoid(NewDirection, Directions, i, OtherRelevantBoids);
		Align(NewDirection, Directions, i, OtherRelevantBoids);
		Constrain(NewDirection, i);

		Directions[i] = NewDirection;
	}

	for (int32 i = 0; i < NumInstances; ++i)
	{
		FTransform Transform{NoInit};
		Mesh->GetInstanceTransform(i, Transform);

		const FVector PreviousLocation = Transform.GetLocation();

		Transform.AddToTranslation(Directions[i] * MovementSpeed * DeltaTime);
		Transform.SetRotation(Directions[i].ToOrientationQuat());
		Mesh->UpdateInstanceTransform(i, Transform);

		RelocateBoidCell(i, PreviousLocation, Transform.GetLocation());
	}
#endif
}

void AFlock::SimulateAsynchronously(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_Simulate_GameThread);

	FMemMark Mark{FMemStack::Get()};

	TArray<FVector, TMemStackAllocator<>> Locations;
	TArray<FVector, TMemStackAllocator<>> Directions;
	{
		DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Initialize Buffers"), STAT_InitializeBuffers, STATGROUP_BoidSimulation);

		Locations.SetNumUninitialized(NumInstances);
		Directions.SetNumUninitialized(NumInstances);

		ParallelFor(NumInstances, [&](const int32 i) -> void
		{
			FTransform Transform{NoInit};
			verify(Mesh->GetInstanceTransform(i, Transform));
			Locations[i] = Transform.GetTranslation();
			Directions[i] = Transform.GetUnitAxis(EAxis::X);
		});
	}

	ParallelFor(NumInstances, [&](const int32 BoidIndex) -> void
	{
		FVector NewDirection = Directions[BoidIndex];// Working with a copy rather than a reference to avoid false-sharing.

		TArray<int32, TInlineAllocator<32>> OtherRelevantBoids;
		{
			SCOPE_CYCLE_COUNTER(STAT_FindNearbyBoids);
			
			ForEachNearbyBoid(Locations[BoidIndex], Locations, [&](const int32 OtherBoidIndex, const FVector& RESTRICT OtherLocation) -> void
			{
				if (BoidIndex == OtherBoidIndex) return;
				
				const FVector Translation = Locations[BoidIndex] - OtherLocation;
				if ((NewDirection | Translation) <= -0.25) return;

				OtherRelevantBoids.Add(OtherBoidIndex);
			});
		}
		
		Cohere(NewDirection, Directions, Locations, BoidIndex, OtherRelevantBoids);
		Avoid(NewDirection, Directions, Locations, BoidIndex, OtherRelevantBoids);
		Align(NewDirection, Directions, Locations, BoidIndex, OtherRelevantBoids);
		Constrain(NewDirection, Locations[BoidIndex], BoidIndex);

		Directions[BoidIndex] = NewDirection;
	});

	// @NOTE: Doesn't scale as well as it should due to the blocking
	ParallelFor(NumInstances, [&](const int32 BoidIndex) -> void
	{
		FVector& RESTRICT Location = Locations[BoidIndex];
		const FVector PreviousLocation = Location;

		Location += Directions[BoidIndex] * MovementSpeed * DeltaTime;
		
		Mesh->UpdateInstanceTransform(BoidIndex, FTransform{Directions[BoidIndex].ToOrientationQuat(), Location});

		SCOPE_CYCLE_COUNTER(STAT_RelocateBoidCells);
		
		{
			const int32 PreviousCellIndex = GetCellIndex(PreviousLocation);
			UE::TScopeLock Lock{BoidCellSpinLocks[PreviousCellIndex]};

			verify(BoidCells[PreviousCellIndex].RemoveSingle(BoidIndex) != INDEX_NONE);

			SCOPE_CYCLE_COUNTER(STAT_RelocateBoidCellsBlockingTime)
		}

		{
			const int32 CellIndex = GetCellIndex(Location);
			UE::TScopeLock Lock{BoidCellSpinLocks[CellIndex]};

			check(!BoidCells[CellIndex].Contains(BoidIndex));
			BoidCells[CellIndex].Add(BoidIndex);

			SCOPE_CYCLE_COUNTER(STAT_RelocateBoidCellsBlockingTime)
		}
	});
}


void AFlock::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidSimulationCVars::EnableMultithreading.GetValueOnGameThread())
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

