// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
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
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TObjectPtr<UInstancedStaticMeshComponent> Mesh;

	void Avoid(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Align(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Cohere(const TArrayView<FVector>& Directions, const int32 BoidIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const;
	void Constrain(FVector& Direction, const int32 BoidIndex) const;

	static constexpr double CELL_SIZE = 25.0;
	TArray<TArray<int32>> BoidCells;

	void SimulateSynchronously(float DeltaTime);
	void SimulateAsynchronously(float DeltaTime);
	
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
};
