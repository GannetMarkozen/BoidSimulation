// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Misc/SpinLock.h"
#include "Flock.generated.h"

class UInstancedStaticMeshComponent;

UCLASS()
class BOIDSIMULATION_API AFlock : public AActor
{
	GENERATED_BODY()
public:
	explicit AFlock(const FObjectInitializer& ObjectInitializer);

protected:
	UPROPERTY(EditAnywhere, Category="Configurations")
	int32 NumInstances = 100;

	UPROPERTY(EditAnywhere, Category="Configurations")
	float BoundsRadius = 1000.f;

	UPROPERTY(EditAnywhere, Category="Configurations")
	float MovementSpeed = 10.f;

	UPROPERTY(EditAnywhere, Category="Configurations")
	float BoidsSearchNearbyRadius = 25.f;

	static constexpr double CELL_SIZE = 125.0;
	TArray<TArray<int32, TInlineAllocator<4>>> BoidCells;
	TArray<UE::FSpinLock> BoidCellSpinLocks;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TObjectPtr<UInstancedStaticMeshComponent> Mesh;

	UE_NODISCARD FORCEINLINE int32 GetHalfCellDimensions() const
	{
		return FMath::CeilToInt32(BoundsRadius / CELL_SIZE);
	}

	UE_NODISCARD FORCEINLINE int32 GetCellDimensions() const
	{
		return GetHalfCellDimensions() * 2;
	}

	UE_NODISCARD FORCEINLINE int32 GetNumCells() const
	{
		return FMath::Cube(GetCellDimensions());
	}

	UE_NODISCARD FORCEINLINE int32 GetAxisCoordinate(const double Value) const
	{
		const int32 HalfCellDimensions = GetHalfCellDimensions();
		return FMath::Clamp(FMath::RoundToInt32(Value / CELL_SIZE) + HalfCellDimensions, 0, (HalfCellDimensions * 2) - 1);
	}

	UE_NODISCARD FORCEINLINE FIntVector GetCellCoordinates(const FVector& Location) const
	{
		const int32 CellDimensions = GetCellDimensions();
		check(CellDimensions % 2 == 0);
		
		const int32 HalfCellDimensions = CellDimensions / 2;

		return FIntVector
		{
			FMath::Clamp(FMath::RoundToInt32(Location.X / CELL_SIZE) + HalfCellDimensions, 0, CellDimensions - 1),
			FMath::Clamp(FMath::RoundToInt32(Location.Y / CELL_SIZE) + HalfCellDimensions, 0, CellDimensions - 1),
			FMath::Clamp(FMath::RoundToInt32(Location.Z / CELL_SIZE) + HalfCellDimensions, 0, CellDimensions - 1)
		};
	}

	UE_NODISCARD FORCEINLINE int32 GetCellIndex(const FIntVector& Coordinates) const
	{
		const int32 CellDimensions = GetCellDimensions();
		return Coordinates.X + Coordinates.Y * CellDimensions + Coordinates.Z * CellDimensions * CellDimensions;
	}

	UE_NODISCARD FORCEINLINE int32 GetCellIndex(const FVector& Location) const
	{
		return GetCellIndex(GetCellCoordinates(Location));
	}

	UE_NODISCARD FORCEINLINE FVector GetCellLocation(const FIntVector& Coordinates) const
	{
		const int32 HalfCellDimensions = FMath::CeilToInt32(BoundsRadius / CELL_SIZE);
		return FVector
		{
			static_cast<double>(Coordinates.X - HalfCellDimensions) * CELL_SIZE,
			static_cast<double>(Coordinates.Y - HalfCellDimensions) * CELL_SIZE,
			static_cast<double>(Coordinates.Z - HalfCellDimensions) * CELL_SIZE
		};
	}
	
	FORCEINLINE void ForEachNearbyBoid(const FVector& RESTRICT Location, const TConstArrayView<FVector>& RESTRICT OtherLocations, const TFunctionRef<void(int32, const FVector&)>& Functor) const
	{
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
					const FIntVector CellCoordinates{X, Y, Z};
					const FVector CellLocation = GetCellLocation(CellCoordinates);
					if (!FMath::SphereAABBIntersection(Location, FMath::Square(static_cast<double>(BoidsSearchNearbyRadius)), FBox{CellLocation - FVector{CELL_SIZE / 2.0}, CellLocation + FVector{CELL_SIZE / 2.0}})) continue;
					
					for (const int32 OtherBoidIndex : BoidCells[GetCellIndex(CellCoordinates)])
					{
						if (FVector::DistSquared(Location, OtherLocations[OtherBoidIndex]) > FMath::Square(BoidsSearchNearbyRadius)) continue;

						Functor(OtherBoidIndex, OtherLocations[OtherBoidIndex]);
					}
				}
			}
		}
	}

	void RelocateBoidCell(const int32 BoidIndex, const FVector& LastLocation, const FVector& NewLocation);

	void Avoid(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Align(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Cohere(FVector& RESTRICT OutDirection, const TConstArrayView<FVector>& Directions, const TConstArrayView<FVector>& Locations, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Constrain(FVector& RESTRICT OutDirection, const FVector& RESTRICT Location, const int32 BoidIndex) const;

	void SimulateSynchronously(float DeltaTime);
	void SimulateAsynchronously(float DeltaTime);
	
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};
