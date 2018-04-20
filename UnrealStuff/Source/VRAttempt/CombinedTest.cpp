// Fill out your copyright notice in the Description page of Project Settings.

#include "CombinedTest.h"
#include "VRAttempt.h" //don't forget to include this!

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

// Creating a standard root object.
ACombinedTest::ACombinedTest()
{
	MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GeneratedMesh"));
	RootComponent = MeshComponent;
	// New in UE 4.17, multi-threaded PhysX cooking.
	MeshComponent->bUseAsyncCooking = true;
	PrimaryActorTick.bCanEverTick = true;
	RefreshRate = 20;
	// Initialize OpenCV and webcam properties
	isStreamOpen = false;
	RefreshTimer = 0.0f;
	ListenSocket = NULL;
	//floodFill variables
	loDiff = 0;
	upDiff = 0;
	connectivity = 4;
	newMaskVal = 255;
	flags = connectivity + (newMaskVal << 8) +
		FLOODFILL_FIXED_RANGE;
	ccomp;
	//dilation variables
	dilation_type = MORPH_RECT;
	dilation_size = 1;
	element = getStructuringElement(dilation_type,
		cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));

	wsize = 15;
	max_disp = 32;
	lambda = 1000;
	sigma = 3.5;
	vis_mult = 3;
}

void ACombinedTest::BeginPlay()
{
	Super::BeginPlay();

	FileStorage fs1("../../../../cam_stereo.yml", cv::FileStorage::READ);
	K1 = (Mat_<double>(3, 3) << 2.5460418200255779e+02, 0., 1.5109157722703864e+02, 0.,
		2.5455332114009897e+02, 1.2303270418075130e+02, 0., 0., 1.);
	K2 = (Mat_<double>(3, 3) << 2.7120710599586960e+02, 0., 1.4924268380330548e+02, 0.,
		2.7152816877324477e+02, 1.1914374359031423e+02, 0., 0., 1.);
	D1 = (Mat_<double>(1, 5) << -1.3275675407751666e-01, 2.7067441536279951e-01,
		-6.3700026239351900e-04, 1.5072067275929610e-03,
		-1.0446086191731643e-01);
	D2 = (Mat_<double>(1, 5) << -1.4712287423352854e-01, 3.0792411282488158e-01,
		-1.1862861126569380e-04, -3.4362557985123665e-03,
		-2.3048600631000718e-01);
	R = (Mat_<double>(3, 3) << 9.9966406126995144e-01, 1.4149014956984372e-02,
		2.1715662113179161e-02, -1.4513008912070881e-02,
		9.9975526541987947e-01, 1.6696761289078755e-02,
		-2.1474104814517492e-02, -1.7006311798075446e-02,
		9.9962475368582271e-01);
	T = { -1.2029936943680282e-01, -6.8099094055332713e-03,
		1.6716444412046082e-02 };

	R1 = (Mat_<double>(3, 3) << 9.9073607827505394e-01, 7.2297029353968589e-02,
		-1.1495722139455898e-01, -7.1418667508982014e-02,
		9.9737725568649283e-01, 1.1746649331696770e-02,
		1.1550496584739423e-01, -3.4277377192230667e-03,
		9.9330098836088987e-01);
	R2 = (Mat_<double>(3, 3) << 9.8892981122750623e-01, 5.5981360953253297e-02,
		-1.3741876033260741e-01, -5.7027683236207716e-02,
		9.9836579374048440e-01, -3.6858151938770258e-03,
		1.3698785278352155e-01, 1.1481686058860598e-02,
		9.9050618325935047e-01);
	P1 = (Mat_<double>(3, 4) << 2.3369219020172852e+02, 0., 1.9210346889495850e+02, 0., 0.,
		2.3369219020172852e+02, 1.1996978187561035e+02, 0., 0., 0., 1.,
		0.);
	P2 = (Mat_<double>(3, 4) << 2.3369219020172852e+02, 0., 1.9210346889495850e+02,
		-2.8427723387849060e+01, 0., 2.3369219020172852e+02,
		1.1996978187561035e+02, 0., 0., 0., 1., 0.);
	Q = (Mat_<double>(4, 4) << 1., 0., 0., -1.9210346889495850e+02, 0., 1., 0.,
		-1.1996978187561035e+02, 0., 0., 0., 2.3369219020172852e+02, 0.,
		0., 8.2205735230143766e+00, 0.);

	receivedImage.create(240, 320, CV_8UC3);
	receivedImage2.create(240, 320, CV_8UC3);
	weighted_map.create(240, 320, CV_64FC1);
	weighted_colour.create(240, 320, CV_32FC3);
	initUndistortRectifyMap(K1, D1, R1, P1, receivedImage.size(), CV_32F, lmapx, lmapy);
	initUndistortRectifyMap(K2, D2, R2, P2, receivedImage2.size(), CV_32F, rmapx, rmapy);
	//namedWindow("Edge Map", CV_WINDOW_AUTOSIZE);

	VideoSize = FVector2D(320, 240);
	VideoTexture = UTexture2D::CreateTransient(VideoSize.X, VideoSize.Y);
	VideoTexture->UpdateResource();
	VideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, VideoSize.X, VideoSize.Y);

	// Initialize data array
	Data.Init(FColor(0, 0, 0, 255), VideoSize.X * VideoSize.Y);

	StartUDPReceiver("fd", "10.9.133.135", 21234);
}

bool ACombinedTest::StartUDPReceiver(
	const FString& YourChosenSocketName,
	const FString& TheIP,
	const int32 ThePort
) {

	//ScreenMsg("RECEIVER INIT");

	//Create Socket
	/*FIPv4Address Addr;
	if (!FIPv4Address::Parse(TheIP, Addr))
	{
		ScreenMsg("Failed to convert IP.");
		return -1;
	}*/

	FIPv4Endpoint Endpoint(FIPv4Address::Any, ThePort);
	RemoteAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

	//BUFFER SIZE
	int32 BufferSize = 40960;

	ListenSocket = FUdpSocketBuilder(*YourChosenSocketName)
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(BufferSize);
	;

	int32 SendSize = 2 * 1024 * 1024;
	ListenSocket->SetSendBufferSize(SendSize, SendSize);

	FTimespan ThreadWaitTime = FTimespan::FromMilliseconds(100);
	UDPReceiver = new FUdpSocketReceiver(ListenSocket, ThreadWaitTime, TEXT("UDP RECEIVER"));
	UDPReceiver->OnDataReceived().BindUObject(this, &ACombinedTest::Recv);
	UDPReceiver->Start();
	return true;
}

void ACombinedTest::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
	//ScreenMsg("Received bytes", ArrayReaderPtr->Num());
	int i = 0;
	do
	{
		int x = (ArrayReaderPtr->GetData()[i * 7] << 8) + ArrayReaderPtr->GetData()[i * 7 + 1];
		int y = (ArrayReaderPtr->GetData()[i * 7 + 2] << 8) + ArrayReaderPtr->GetData()[i * 7 + 3];
		Point seed = Point(x, y);
		Scalar newVal = Scalar(ArrayReaderPtr->GetData()[i * 7 + 4], ArrayReaderPtr->GetData()[i * 7 + 5], ArrayReaderPtr->GetData()[i * 7 + 6]);
		//ScreenMsg("SeedPoint: ",MakeSeedData(seed, newVal) );
		seedList.push_back(MakeSeedData(seed, newVal));
		i++;
		if (i > 200) {
			ScreenMsg("Colours-Image1 splitter not found\n");
			seedList.clear();
			imageBuf.clear();
			image2Buf.clear();
			return;
		}
	} while (!(ArrayReaderPtr->GetData()[i * 7 + 0] == 0 && ArrayReaderPtr->GetData()[i * 7 + 1] == 1
		&& ArrayReaderPtr->GetData()[i * 7 + 2] == 2 && ArrayReaderPtr->GetData()[i * 7 + 3] == 3
		&& ArrayReaderPtr->GetData()[i * 7 + 4] == 4 && ArrayReaderPtr->GetData()[i * 7 + 5] == 5
		&& ArrayReaderPtr->GetData()[i * 7 + 6] == 6)); // CHANGE BACK TO INCREMENTING FOR SYSTEM TEST
	i++;
	i = i * 7;
	do
	{
		imageBuf.push_back(ArrayReaderPtr->GetData()[i]);
		i++;
		if (i > 10000) {
			ScreenMsg("Image1-Image2 splitter not found\n");
			seedList.clear();
			imageBuf.clear();
			image2Buf.clear();
			return;
		}
	} while (!(ArrayReaderPtr->GetData()[i] == 0 && ArrayReaderPtr->GetData()[i + 1] == 1
		&& ArrayReaderPtr->GetData()[i + 2] == 2 && ArrayReaderPtr->GetData()[i + 3] == 3
		&& ArrayReaderPtr->GetData()[i + 4] == 4 && ArrayReaderPtr->GetData()[i + 5] == 5
		&& ArrayReaderPtr->GetData()[i + 6] == 6));
	i += 7;
	do
	{
		image2Buf.push_back(ArrayReaderPtr->GetData()[i]);
		i++;
	} while (i != ArrayReaderPtr->Num());

	receivedImage = imdecode(imageBuf, IMREAD_COLOR);
	if (receivedImage.empty()) {
		ScreenMsg("Images not received\n");
		seedList.clear();
		imageBuf.clear();
		image2Buf.clear();
		return;
	}
	receivedImage2 = imdecode(image2Buf, IMREAD_COLOR);
	if (receivedImage2.empty()) {
		ScreenMsg("Only one image received\n");
		seedList.clear();
		imageBuf.clear();
		image2Buf.clear();
		return;
	}

	colouredImage = receivedImage.clone();
	dilate(colouredImage, colouredImage, element);
	for (int j = 0; j < seedList.size(); j++)
	{
		//seedList[j].seed.x = seedList[j].seed.x*2;
		//seedList[j].seed.y = seedList[j].seed.y*2;
		if (colouredImage.at<Vec3b>(seedList[j].seed)[0] == 0 &&
			colouredImage.at<Vec3b>(seedList[j].seed)[1] == 0 &&
			colouredImage.at<Vec3b>(seedList[j].seed)[2] == 0)
		{
			floodFill(colouredImage, seedList[j].seed, seedList[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
				Scalar(upDiff, upDiff, upDiff), flags);
		}
		//circle(colouredImage, seedList[j].seed, 5, (0,0,255), -1);
	}
	/*for (int i = 0; i<colouredImage.rows; i++) {
	for (int j = 0; j<colouredImage.cols; j++) {
	if (colouredImage.at<Vec3b>(i, j)[0] == 0) colouredImage.at<Vec3b>(i, j) = Vec3b(255, 0, 255);
	}
	}*/
	remap(colouredImage, colouredImage, lmapx, lmapy, cv::INTER_LINEAR);
	remap(receivedImage, receivedImage, lmapx, lmapy, cv::INTER_LINEAR);
	remap(receivedImage2, receivedImage2, rmapx, rmapy, cv::INTER_LINEAR);

	left_for_matcher = receivedImage.clone();
	right_for_matcher = receivedImage2.clone();

	Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
	wls_filter = createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

	cvtColor(left_for_matcher, left_for_matcher, COLOR_BGR2GRAY);
	cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
	left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
	right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->filter(left_disp, left_for_matcher, filtered_disp, right_disp);
	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);

	accumulateWeighted(filtered_disp_vis, weighted_map, 0.3);
	convertScaleAbs(weighted_map, output_map);
	//imshow("Edge Map", output_map);
	if (!isStreamOpen) {
		isStreamOpen = true;
		bool bIsValid;
		FString SendIP = EndPt.Address.ToString();
		RemoteAddr->SetIp(*SendIP, bIsValid);
		RemoteAddr->SetPort(EndPt.Port);
		if (!bIsValid)
		{
			ScreenMsg("Rama UDP Sender>> IP address was not valid!");
		}
	}
	
		
	UpdateTexture();
	seedList.clear();
	imageBuf.clear();
	image2Buf.clear();
}

void ACombinedTest::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	//~~~~~~~~~~~~~~~~
	UDPReceiver->Stop();
	delete UDPReceiver;
	UDPReceiver = nullptr;

	//Clear all sockets!
	//		makes sure repeat plays in Editor dont hold on to old sockets!
	if (ListenSocket)
	{
		ListenSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
	}
}


// Called every frame
void ACombinedTest::Tick(float DeltaSeconds)
{
	RefreshTimer += DeltaSeconds;
	if (isStreamOpen && RefreshTimer >= 1.0f / RefreshRate)
	{
		RefreshTimer -= 1.0f / RefreshRate;
		OnNextVideoFrame();
		if (AnimateMesh)
		{
			CurrentAnimationFrameX += DeltaSeconds * AnimationSpeedX;
			CurrentAnimationFrameY += DeltaSeconds * AnimationSpeedY;
			GenerateMesh();
		}
	}
	if (isStreamOpen && RefreshTimer >= 1.0f / 50) {
		Super::Tick(DeltaSeconds);
		SendUDP();
	}
}

bool ACombinedTest::SendUDP()
{
	if (!ListenSocket)
	{
		ScreenMsg("No sender socket");
		return false;
	}
	//~~~~~~~~~~~~~~~~

	int32 BytesSent = 0;
	uint8 packet[6];
	packet[0] = DriveX;
	packet[1] = DriveY;
	packet[2] = lowThreshold;
	packet[3] = GimbleX;
	packet[4] = GimbleY;
	packet[5] = 0;

	bool success = ListenSocket->SendTo(packet, 6, BytesSent, *RemoteAddr);

	if(!success || BytesSent < 0) 
	{
		const FString Str = "Socket is valid but the receiver received 0 bytes, make sure it is listening properly!";
		UE_LOG(LogTemp, Error, TEXT("%s"), *Str);
		ScreenMsg(Str);
		return false;
	}
	//ScreenMsg("UDP~ Send Succcess! Bytes Sent = ", packet[0]);

	return true;
}

void ACombinedTest::UpdateTexture()
{
	if (isStreamOpen && colouredImage.data)
	{
		// Copy Mat data to Data array
		for (int y = 0; y < VideoSize.Y; y++)
		{
			for (int x = 0; x < VideoSize.X; x++)
			{
				int i = x + (y * VideoSize.X);
				Data[i].B = colouredImage.data[i * 3 + 0];
				Data[i].G = colouredImage.data[i * 3 + 1];
				Data[i].R = colouredImage.data[i * 3 + 2];
			}
		}

		// Update texture 2D
		UpdateTextureRegions(VideoTexture, (int32)0, (uint32)1, VideoUpdateTextureRegion, (uint32)(4 * VideoSize.X), (uint32)4, (uint8*)Data.GetData(), false);
	}
}

// This is called when actor is spawned (at runtime or when you drop it into the world in editor)
void ACombinedTest::PostActorCreated()
{
	Super::PostActorCreated();
	GenerateMesh();
}

// This is called when actor is already in level and map is opened
void ACombinedTest::PostLoad()
{
	Super::PostLoad();
	GenerateMesh();
}

void ACombinedTest::SetupMeshBuffers()
{
	int32 NumberOfPoints = (LengthSections + 1) * (WidthSections + 1);
	int32 NumberOfTriangles = LengthSections * WidthSections * 2 * 3; // 2x3 vertex indexes per quad
	Vertices.AddUninitialized(NumberOfPoints);
	normals.AddUninitialized(NumberOfPoints);
	UV0.AddUninitialized(NumberOfPoints);
	Triangles.AddUninitialized(NumberOfTriangles);
	HeightValues.AddUninitialized(NumberOfPoints);
}

void ACombinedTest::GeneratePoints()
{
	// Setup example height data
	int32 NumberOfPoints = (LengthSections + 1) * (WidthSections + 1);
	float AvgValue;
	// Combine variations of sine and cosine to create some variable waves
	// TODO Convert this to use a parallel for
	int32 PointIndex = 0;
	int32 mat_X = 239;
	int32 mat_Y = 319;

	for (int32 X = 0; X < LengthSections + 1; X++)
	{
		for (int32 Y = 0; Y < WidthSections + 1; Y++)
		{
			// Just some quick hardcoded offset numbers in there
			//float ValueOne = FMath::Cos((X + CurrentAnimationFrameX)*ScaleFactor) * FMath::Sin((Y + CurrentAnimationFrameY)*ScaleFactor);
			//float ValueTwo = FMath::Cos((X + CurrentAnimationFrameX*0.7f)*ScaleFactor*2.5f) * FMath::Sin((Y - CurrentAnimationFrameY*0.7f)*ScaleFactor*2.5f);
			//AvgValue = ((ValueOne + ValueTwo) / 2) * Size.Z;
			if (isStreamOpen) {
				//ScreenMsg("X,Y = ", output_map.at<uchar>(X, Y));
				AvgValue = output_map.at<uchar>(Y, X);
			}
			//else AvgValue = 0; 
			HeightValues[PointIndex++] = 8 * AvgValue;

			if (AvgValue > MaxHeightValue)
			{
				MaxHeightValue = AvgValue;
			}
			mat_Y--;
		}
		mat_X--;
	}
}

void ACombinedTest::GenerateMesh()
{
	if (Size.X < 1 || Size.Y < 1 || LengthSections < 1 || WidthSections < 1)
	{
		MeshComponent->ClearAllMeshSections();
		return;
	}

	// The number of vertices or polygons wont change at runtime, so we'll just allocate the arrays once
	if (!bHaveBuffersBeenInitialized)
	{
		SetupMeshBuffers();
		bHaveBuffersBeenInitialized = true;
	}

	GeneratePoints();

	// TODO Convert this to use fast-past updates instead of regenerating the mesh every frame
	GenerateGrid(Vertices, Triangles, UV0, FVector2D(Size.X, Size.Y), LengthSections, WidthSections, HeightValues);
	MeshComponent->ClearAllMeshSections();
	MeshComponent->CreateMeshSection(0, Vertices, Triangles, TArray<FVector>(), UV0, TArray<FColor>(), TArray<FProcMeshTangent>(), false);
	//MeshComponent->SetMaterial(0, Material);
}

void ACombinedTest::GenerateGrid(TArray<FVector>& InVertices, TArray<int32>& InTriangles, TArray<FVector2D>& InUV0, FVector2D InSize, int32 InLengthSections, int32 InWidthSections, const TArray<float>& InHeightValues)
{
	// Note the coordinates are a bit weird here since I aligned it to the transform (X is forwards or "up", which Y is to the right)
	// Should really fix this up and use standard X, Y coords then transform into object space?
	FVector2D SectionSize = FVector2D(InSize.X / InLengthSections, InSize.Y / InWidthSections);
	int32 VertexIndex = 0;
	int32 TriangleIndex = 0;

	for (int32 X = 0; X < InLengthSections + 1; X++)
	{
		for (int32 Y = 0; Y < InWidthSections + 1; Y++)
		{
			// Create a new vertex
			int32 NewVertIndex = VertexIndex++;
			FVector newVertex = FVector(X * SectionSize.X, Y * SectionSize.Y, InHeightValues[NewVertIndex]);
			InVertices[NewVertIndex] = newVertex;

			// Note that Unreal UV origin (0,0) is top left
			float U = (float)X / (float)InLengthSections;
			float V = (float)Y / (float)InWidthSections;
			InUV0[NewVertIndex] = FVector2D(U, V);

			// Once we've created enough verts we can start adding polygons
			if (X > 0 && Y > 0)
			{
				// Each row is InWidthSections+1 number of points.
				// And we have InLength+1 rows
				// Index of current vertex in position is thus: (X * (InWidthSections + 1)) + Y;
				int32 bTopRightIndex = (X * (InWidthSections + 1)) + Y; // Should be same as VertIndex1!
				int32 bTopLeftIndex = bTopRightIndex - 1;
				int32 pBottomRightIndex = ((X - 1) * (InWidthSections + 1)) + Y;
				int32 pBottomLeftIndex = pBottomRightIndex - 1;

				// Now create two triangles from those four vertices
				// The order of these (clockwise/counter-clockwise) dictates which way the normal will face. 
				InTriangles[TriangleIndex++] = pBottomLeftIndex;
				InTriangles[TriangleIndex++] = bTopRightIndex;
				InTriangles[TriangleIndex++] = bTopLeftIndex;

				InTriangles[TriangleIndex++] = pBottomLeftIndex;
				InTriangles[TriangleIndex++] = pBottomRightIndex;
				InTriangles[TriangleIndex++] = bTopRightIndex;

				// Normals
				//FVector NormalCurrent = FVector::CrossProduct(InVertices[pBottomLeftIndex] - InVertices[bTopLeftIndex], InVertices[bTopLeftIndex] - InVertices[bTopRightIndex]).GetSafeNormal();
				//InNormals[pBottomLeftIndex] = InNormals[pBottomRightIndex] = InNormals[bTopRightIndex] = InNormals[bTopLeftIndex] = FPackedNormal(NormalCurrent);
			}
		}
	}
}

#if WITH_EDITOR
void ACombinedTest::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	FName MemberPropertyChanged = (PropertyChangedEvent.MemberProperty ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None);

	if ((MemberPropertyChanged == GET_MEMBER_NAME_CHECKED(ACombinedTest, Size)) || (MemberPropertyChanged == GET_MEMBER_NAME_CHECKED(ACombinedTest, ScaleFactor)))
	{
		// Same vert count, so just regen mesh with same buffers
		GenerateMesh();
	}
	else if ((MemberPropertyChanged == GET_MEMBER_NAME_CHECKED(ACombinedTest, LengthSections)) || (MemberPropertyChanged == GET_MEMBER_NAME_CHECKED(ACombinedTest, WidthSections)))
	{
		// Vertice count has changed, so reset buffer and then regen mesh
		Vertices.Empty();
		Triangles.Empty();
		HeightValues.Empty();
		bHaveBuffersBeenInitialized = false;
		GenerateMesh();
	}
	//else if ((MemberPropertyChanged == GET_MEMBER_NAME_CHECKED(ACombinedTest, Material)))
	//{
	//	MeshComponent->SetMaterial(0, Material);
	//}
}
#endif // WITH_EDITOR

void ACombinedTest::UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData)
{
	if (Texture->Resource)
	{
		struct FUpdateTextureRegionsData
		{
			FTexture2DResource* Texture2DResource;
			int32 MipIndex;
			uint32 NumRegions;
			FUpdateTextureRegion2D* Regions;
			uint32 SrcPitch;
			uint32 SrcBpp;
			uint8* SrcData;
		};

		FUpdateTextureRegionsData* RegionData = new FUpdateTextureRegionsData;

		RegionData->Texture2DResource = (FTexture2DResource*)Texture->Resource;
		RegionData->MipIndex = MipIndex;
		RegionData->NumRegions = NumRegions;
		RegionData->Regions = Regions;
		RegionData->SrcPitch = SrcPitch;
		RegionData->SrcBpp = SrcBpp;
		RegionData->SrcData = SrcData;

		ENQUEUE_UNIQUE_RENDER_COMMAND_TWOPARAMETER(
			UpdateTextureRegionsData,
			FUpdateTextureRegionsData*, RegionData, RegionData,
			bool, bFreeData, bFreeData,
			{
				for (uint32 RegionIndex = 0; RegionIndex < RegionData->NumRegions; ++RegionIndex)
				{
					int32 CurrentFirstMip = RegionData->Texture2DResource->GetCurrentFirstMip();
					if (RegionData->MipIndex >= CurrentFirstMip)
					{
						RHIUpdateTexture2D(
							RegionData->Texture2DResource->GetTexture2DRHI(),
							RegionData->MipIndex - CurrentFirstMip,
							RegionData->Regions[RegionIndex],
							RegionData->SrcPitch,
							RegionData->SrcData
							+ RegionData->Regions[RegionIndex].SrcY * RegionData->SrcPitch
							+ RegionData->Regions[RegionIndex].SrcX * RegionData->SrcBpp
						);
					}
				}
		if (bFreeData)
		{
			FMemory::Free(RegionData->Regions);
			FMemory::Free(RegionData->SrcData);
		}
		delete RegionData;
			});
	}
}