// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Networking.h"
#include "Transmission.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VRATTEMPT_API UTransmission : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UTransmission();

	TSharedPtr<FInternetAddr>	RemoteAddr;
	FSocket* SenderSocket;

	bool StartUDPSender(
		const FString& YourChosenSocketName,
		const FString& TheIP,
		const int32 ThePort
	);
	bool RamaUDPSender_SendString();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		bool ShowOnScreenDebugMessages;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 DriveX;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 DriveY;

	int32 sendRate;
	bool isSending;
	float RefreshTimer;

	//ScreenMsg
	FORCEINLINE void ScreenMsg(const FString& Msg)
	{
		if (!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, *Msg);
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const float Value)
	{
		if (!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %f"), *Msg, Value));
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const FString& Msg2)
	{
		if (!ShowOnScreenDebugMessages) return;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %s"), *Msg, *Msg2));
	}

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Called whenever this actor is being removed from a level */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	
};
