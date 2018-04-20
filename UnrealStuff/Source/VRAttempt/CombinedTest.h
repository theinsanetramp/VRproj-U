// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"	
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "CoreMinimal.h"
#include "Networking.h"
#include "GameFramework/Actor.h"
#include "Runtime/Engine/Classes/Engine/Texture2D.h"
#include "ProceduralMeshComponent.h"
#include "CombinedTest.generated.h"

USTRUCT()
struct FSeedData
{
	GENERATED_BODY()

		cv::Point seed;

	cv::Scalar colour;
};

UCLASS()
class VRATTEMPT_API ACombinedTest : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ACombinedTest();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// The rate at which the color data array and video texture is updated (in frames per second)
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = Webcam)
		float RefreshRate;

	// The refresh timer
	UPROPERTY(BlueprintReadWrite, Category = Webcam)
		float RefreshTimer;

	// Blueprint Event called every time the video frame is updated
	UFUNCTION(BlueprintImplementableEvent, Category = Webcam)
		void OnNextVideoFrame();

	// OpenCV fields
	cv::Mat receivedImage, colouredImage;
	cv::Mat receivedImage2, weighted_colour;
	//floodFill variables
	int loDiff, upDiff;
	int connectivity;
	int newMaskVal;
	int flags;
	cv::Rect ccomp;
	//dilation variables
	int dilation_type;
	int dilation_size;
	cv::Mat element;

	// OpenCV prototypes
	void UpdateTexture();
	FSeedData MakeSeedData(cv::Point seed, cv::Scalar colour)
	{
		FSeedData s;
		s.seed = seed;
		s.colour = colour;
		return s;
	}
	std::vector<FSeedData> seedList;
	std::vector<uchar> imageBuf;
	std::vector<uchar> image2Buf;

	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat K1, K2, R;
	cv::Vec3d T;
	cv::Mat D1, D2;
	cv::Mat lmapx, lmapy, rmapx, rmapy;

	int wsize;
	int max_disp;
	double lambda;
	double sigma;
	double vis_mult;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	cv::Mat left_for_matcher, right_for_matcher;
	cv::Mat filtered_disp_vis;
	cv::Mat left_disp, right_disp;
	cv::Mat filtered_disp;
	cv::Mat weighted_map, output_map;

	// If the stream has succesfully opened yet
	UPROPERTY(BlueprintReadOnly, Category = Webcam)
		bool isStreamOpen;

	// The videos width and height (width, height)
	UPROPERTY(BlueprintReadWrite, Category = Webcam)
		FVector2D VideoSize;

	// The current video frame's corresponding texture
	UPROPERTY(BlueprintReadOnly, Category = Webcam)
		UTexture2D* VideoTexture;

	// The current data array
	UPROPERTY(BlueprintReadOnly, Category = Webcam)
		TArray<FColor> Data;

	FSocket* ListenSocket;
	FUdpSocketReceiver* UDPReceiver = nullptr;
	void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);
	TSharedPtr<FInternetAddr>	RemoteAddr;

	bool StartUDPReceiver(
		const FString& YourChosenSocketName,
		const FString& TheIP,
		const int32 ThePort
	);

	FORCEINLINE void ScreenMsg(const FString& Msg)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, *Msg);
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const float Value)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s %.2f"), *Msg, Value));
	}
	FORCEINLINE void ScreenMsg(const FString& Msg, const FSeedData SeedPoint)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString::Printf(TEXT("%s [%d,%d] [%.0f,%.0f,%.0f]"), *Msg, SeedPoint.seed.x, SeedPoint.seed.y, SeedPoint.colour[0], SeedPoint.colour[1], SeedPoint.colour[2]));
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 DriveX;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 DriveY;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 lowThreshold = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 GimbleX;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rama UDP Sender")
		int32 GimbleY;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(BlueprintReadWrite, Category = "Procedural Parameters")
		UProceduralMeshComponent * MeshComponent;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		FVector Size = FVector(1000.0f, 1000.0f, 100.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		float ScaleFactor = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		int32 LengthSections = 319;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		int32 WidthSections = 239;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		UMaterialInterface* Material;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		bool AnimateMesh = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		float AnimationSpeedX = 4.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Parameters")
		float AnimationSpeedY = 4.5f;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif // WITH_EDITOR

protected:
	virtual void PostActorCreated() override;
	virtual void PostLoad() override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Default)
		USceneComponent* RootNode;

	float CurrentAnimationFrameX = 0.0f;
	float CurrentAnimationFrameY = 0.0f;

	void UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData);

	// Pointer to update texture region 2D struct
	FUpdateTextureRegion2D* VideoUpdateTextureRegion;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	bool SendUDP();
private:

	void GenerateMesh();
	void GeneratePoints();
	void GenerateGrid(TArray<FVector>& InVertices, TArray<int32>& InTriangles, TArray<FVector2D>& InUV0, FVector2D InSize, int32 InLengthSections, int32 InWidthSections, const TArray<float>& InHeightValues);

	TArray<float> HeightValues;
	float MaxHeightValue = 0.0f;

	// Mesh buffers
	void SetupMeshBuffers();
	bool bHaveBuffersBeenInitialized = false;
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> normals;
	TArray<FVector2D> UV0;

};
