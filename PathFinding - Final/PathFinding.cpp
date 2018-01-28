// PathFinding.cpp: A program using the TL-Engine
// Daniel MacArthur

#include <TL-Engine.h>	
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include <random>

using namespace tle;
I3DEngine* myEngine = New3DEngine(kTLX);

const float SearchTime = 0.2f;
const float PlayerSpeed = 10.0f;
float kPlayerSpeed = 1.0f;

//Globals 
enum Types { Wall, Clear, Wood, Water };

//Map
const int gMapWidth = 10;
const int gMapHeight = 10;

//Nodes
IModel * gModelRed[gMapWidth][gMapHeight]; //Array of Finding Nodes
IModel * gModelGreen[gMapWidth][gMapHeight]; //Array of Current nodes
const float NodeHeight = 10.0f;
const float CubeSize = 10.0f;

//Array of Spline Cubes
int gCurveCounter = 0;
const int CurveDensity = 40;
const float PathRadius = 0.1f;
IModel* Curve[10000];

struct Node
{
	int x, y, ManDistance = 0, Weight = 0, TotalScore = 0;
	shared_ptr<Node> Parent;
};

struct Coords
{
	int x;
	int y;
};

vector <Coords> RedList;
vector <Coords> GreenList;
vector <shared_ptr<Node> > OpenList;
vector <shared_ptr<Node> > CloseList;

//The Bezier Equation
float Bezier(float P1, float P2, float P3, float P4, float t)
{
	return pow((1 - t), 3) * P1 + 3 * t * pow((1 - t), 2) * P2 + 3 * pow(t, 2) * (1 - t) * P3 + pow(t, 3) * P4;
}

void GenMap()
{
	default_random_engine generator;
	uniform_int_distribution<int> distributionn(0, 2);

	int MapGenArray[gMapWidth][gMapHeight];
	for (int i = 0; i < gMapWidth; i++)
	{
		for (int j = 0; j < gMapHeight; j++)
		{
			int dice_roll = distributionn(generator);
			MapGenArray[i][j] = dice_roll;
		}
	}
	ofstream OutputC("0map.txt");

	for (int i = 0; i < gMapWidth; i++)
	{
		for (int j = 0; j < gMapHeight; j++)
		{
			OutputC << MapGenArray[i][j];
		}
		OutputC << endl;
	}
	OutputC.close(); // Close the file

	ofstream OutputM("0coords.txt");

	uniform_int_distribution<int> distribution(0, 9);
	int dice_roll = distribution(generator);
	OutputM << dice_roll << " " ;
	dice_roll = distribution(generator);
	OutputM << dice_roll << " " << endl;
	dice_roll = distribution(generator);
	OutputM << dice_roll << " ";
	dice_roll = distribution(generator);
	OutputM << dice_roll << " " << endl;

	OutputM.close(); // Close the file

}

// Reads in a map & Coord file: (MapArray, Name of map)
bool GetMap(int MapArray[gMapWidth][gMapHeight], string MapFile, shared_ptr<Node> Start, shared_ptr<Node> Goal)
{
	if (MapFile == "0") GenMap();

	//Read in Map file Given its prefix
	string CoordFile;
	CoordFile = MapFile + "Coords.txt";
	MapFile = MapFile + "map.txt";
	ifstream MapFileIn(MapFile);

	// Error catcher for missing map
	if (!MapFileIn)
	{
		cout << "ERROR: ";
		cout << "Can't open file " + MapFile + ", File May be missing... \n";
		return false;
	}

	// If file was read succesfully, write the numbers to an array
	else {

		// Read map variables
		MapFileIn >> noskipws;
		int CounterX = 0;					// Counting along X axis
		int CounterY = (gMapHeight - 1);		// Offset so that (x,y) co-ordinates match Array[X][Y] 

		while (!MapFileIn.eof())
		{
			// Reads each character of the file
			char ch;
			MapFileIn >> ch;

			//Adding Numbers to array
			if (ch != 10) {
				int MapInt = (int)ch - '0';		// Convert ch to int to be added to the array
				MapArray[CounterX][CounterY] = MapInt;  // Add the int to the Array

				//When The X counter reaches the right hand side, reset to the left hand side and starts the next line
				CounterX++;
				if (CounterX > (gMapWidth - 1)) {
					CounterX = 0;
					CounterY--;
				}
			}
		}

		// Confirm map has loaded
		cout << "[INFO] " << MapFile << " Successfully Loaded." << endl;
	}

	// Read In given map
	ifstream CoordFileIn(CoordFile);

	// Error catcher for missing map
	if (!CoordFileIn)
	{
		cout << "ERROR: ";
		cout << "Can't open file " + CoordFile + ", File May be missing... \n";
		return false;
	}

	else {

		// Read map variables
		MapFileIn >> noskipws;
		int CoordArray[5];
		int i = 0;
		while (!CoordFileIn.eof())
		{
			// Reads each character of the file
			char ch;
			CoordFileIn >> ch;

			//Adding Numbers to array
			if (ch != 10)
			{
				int MapInt = (int)ch - '0';		// Convert ch to int to be added to the array
				CoordArray[i] = MapInt;
				i++;
			}
		}

		//Setup Start and End Nodes
		Start->x = CoordArray[0];
		Start->y = CoordArray[1];
		Goal->x = CoordArray[2];
		Goal->y = CoordArray[3];

		cout << "[INFO] " << CoordFile << " Successfully Loaded." << endl; // Confirm map has loaded
		return true;
	}
}

// Prints the current map: (MapArray)
void DisplayMap(int a[gMapWidth][gMapHeight])
{
	for (int i = 0; i < gMapHeight; i++) //loops though the collums
	{
		for (int j = 0; j < gMapWidth; j++) //loops though the rows
		{
			cout << a[j][(gMapWidth - 1) - i] << " "; //prints The correct way up
		}
		cout << endl;
	}
}

//Check Close List
bool CheckCloseList(shared_ptr<Node> Current, IMesh* CubeMesh)
{

	for (int j = 0; j < OpenList.size(); j++)
	{
		if (Current->x == OpenList[j]->x && Current->y == OpenList[j]->y)
		{
			return true;
		}
	}

	for (int j = 0; j < CloseList.size(); j++)
	{
		if (Current->x == CloseList[j]->x && Current->y == CloseList[j]->y)
		{
			//If the Total score is bigger than The one on the close list ignore it
			if (Current->TotalScore >= CloseList[j]->TotalScore) 
			{
				return true;
			}

			else //if not then get rid of the one on the close list and add this one to the open list again
			{
				//Keeps a record of what tiles have been created to be deleted later
				Coords Red = { Current->x, Current->y };
				for (int i = 0; i < RedList.size(); i++)
				{
					//if the model was created already, delete it so not to cause over lapping
					if (RedList[i].x == Current->x && RedList[i].y == Current->y)
					{
						RedList.erase(RedList.begin() + i);
						CubeMesh->RemoveModel(gModelRed[Current->x][Current->y]);
					}
				}
				RedList.push_back(Red);

				//Create the Model
				gModelRed[Current->x][Current->y] = CubeMesh->CreateModel(CubeSize * Current->x, NodeHeight, CubeSize * Current->y);
				gModelRed[Current->x][Current->y]->SetSkin("RedBall.jpg");
				gModelRed[Current->x][Current->y]->Scale(0.2f);
				myEngine->DrawScene();
				OpenList.push_back(Current);
				CloseList.erase(CloseList.begin() + j); //Get rid of from list

				return true;
			}
		}
	}
	return false;
}

void CreateNode(int x, int y, shared_ptr<Node> Goal, shared_ptr<Node> Previous, int Map[gMapWidth][gMapHeight], float myTimer, IMesh* CubeMesh)
{
	shared_ptr<Node> NewNode(new Node);
	NewNode->x = x;
	NewNode->y = y;
	NewNode->Weight = Map[x][y];
	NewNode->Parent = Previous;
	NewNode->ManDistance = abs(NewNode->x - Goal->x) + abs(NewNode->y - Goal->y);
	NewNode->TotalScore = Previous->TotalScore + NewNode->ManDistance + NewNode->Weight - Previous->ManDistance;

	if (NewNode->Weight == 0)
	{
		cout << " Wall" << endl;
		return; //Node is a wall
	}
	if (CheckCloseList(NewNode, CubeMesh))
	{
		return; //Node already exists
	}

	while (myTimer < SearchTime)
	{
		myTimer = myTimer + myEngine->Timer();
	}
	myTimer = 0;
	cout << "Node found: " << NewNode->x << ", " << NewNode->y << endl;

	Coords Red = { x, y };
	for (int i = 0; i < RedList.size(); i++)
	{
		if (RedList[i].x == x && RedList[i].y == y)
		{
			RedList.erase(RedList.begin() + i);
			CubeMesh->RemoveModel(gModelRed[x][y]);
		}
	}
	RedList.push_back(Red);

	gModelRed[x][y] = CubeMesh->CreateModel(CubeSize * x, NodeHeight, CubeSize * y);
	gModelRed[x][y]->SetSkin("RedBall.jpg");
	gModelRed[x][y]->Scale(0.2f);
	myEngine->DrawScene();
	OpenList.push_back(NewNode); //Add to close list

}

void QuickSort(vector <shared_ptr<Node>> &List, int left, int right) {

	int i = left, j = right;
	shared_ptr<Node> tmp(new Node);
	int pivot = List[(left + right) / 2]->TotalScore;

	//partition 
	while (i <= j)
	{
		while (List[i]->TotalScore < pivot)
			i++;

		while (List[j]->TotalScore > pivot)
			j--;

		if (i <= j)
		{
			tmp = List[i];
			List[i] = List[j];
			List[j] = tmp;
			i++;
			j--;
		}
	};

	//recursion

	if (left < j)
		QuickSort(List, left, j);

	if (i < right)
		QuickSort(List, i, right);

}

void Delete(IMesh * CubeMesh)
{
	//Deletes Greens
	while (!GreenList.empty())
	{
		CubeMesh->RemoveModel(gModelGreen[GreenList[0].x][GreenList[0].y]);
		GreenList.erase(GreenList.begin());
	}

	//Deletes reds
	while (!RedList.empty())
	{
		CubeMesh->RemoveModel(gModelRed[RedList[0].x][RedList[0].y]);
		RedList.erase(RedList.begin());
		myEngine->DrawScene();
	}
}

void reconstructpath(shared_ptr<Node> current, shared_ptr<Node> Start, IMesh * CubeMesh, float myTimer, string MapFile)
{
	vector <shared_ptr<Node> >  FinalPath;

	while (current->x != Start->x || current->y != Start->y)
	{
		FinalPath.insert(FinalPath.begin(), current);
		for (int i = 0; i < CloseList.size(); i++)
		{
			if (current->Parent->x == CloseList[i]->x &&  current->Parent->y == CloseList[i]->y)
			{
				current = CloseList[i];
				i = CloseList.size() - 1;
			}
		}
	}

	FinalPath.insert(FinalPath.begin(), Start);
	cout << "Route Found { ";

	//Deletes Greens
	while (!GreenList.empty())
	{
		CubeMesh->RemoveModel(gModelGreen[GreenList[0].x][GreenList[0].y]);
		GreenList.erase(GreenList.begin());
	}

	//Deletes reds
	while (!RedList.empty())
	{
		CubeMesh->RemoveModel(gModelRed[RedList[0].x][RedList[0].y]);
		RedList.erase(RedList.begin());
		myEngine->DrawScene();
	}

	//Output the final path to text file
	ofstream Output("output.txt");
	Output << "Path for " << MapFile << "Map.txt" << endl;
	for (int i = 0; i < FinalPath.size(); i++)
	{
		cout << "(" << FinalPath[i]->x << ", " << FinalPath[i]->y << ")" << endl ;
		Output << "(" << FinalPath[i]->x << ", " << FinalPath[i]->y << ") " << endl ;
	}
	cout << " }" << endl;
	Output.close(); // Close the file

	//Smooth Curve 
	float P1x;
	float P2x;
	float P3x;
	float P4x;
	float X;
	float P1y;
	float P2y;
	float P3y;
	float P4y;
	float Y;
	float fCurve = CurveDensity;
	int NumLeft = FinalPath.size();

	for (int i = 0; i < FinalPath.size() - 3; )
	{
		P1x = FinalPath[i]->x;
		P2x = FinalPath[i + 1]->x;
		P3x = FinalPath[i + 2]->x;
		P4x = FinalPath[i + 3]->x;
		P1y = FinalPath[i]->y;
		P2y = FinalPath[i + 1]->y;
		P3y = FinalPath[i + 2]->y;
		P4y = FinalPath[i + 3]->y;

		//If there are 4 points to connect
		if (P1x != P4x && P1y != P4y)
		{
			i = i + 3;
			for (int j = 0; j < (CurveDensity); j++)
			{
				X = Bezier(P1x, P2x, P3x, P4x, j / fCurve);
				Y = Bezier(P1y, P2y, P3y, P4y, j / fCurve);

				//Models
				Curve[gCurveCounter] = CubeMesh->CreateModel(CubeSize * X, 11.0f, CubeSize * Y);
				Curve[gCurveCounter]->SetSkin("RedBall.jpg");
				Curve[gCurveCounter]->Scale(0.1f);
				myEngine->DrawScene();
				gCurveCounter++;
			}
		}

		//Straight parts
		else
		{
			for (int j = 0; j < (CurveDensity / 4 ); j++)
			{
				//Draw along the Y Axis
				if (P1y != P2y)
				{
					if (P1y < P2y)
					{
						X = FinalPath[i]->x;
						Y = FinalPath[i]->y + (j / (fCurve / 4.0f));
					}

					else
					{
						X = FinalPath[i]->x;
						Y = FinalPath[i]->y - (j / (fCurve / 4.0f));
					}
				}

				//Draw along X
				else
				{
					if (P1x < P2x)
					{
						X = FinalPath[i]->x + (j / (fCurve / 4.0f));
						Y = FinalPath[i]->y;
					}
					else
					{
						X = FinalPath[i]->x - (j / (fCurve / 4.0f));
						Y = FinalPath[i]->y;
					}
				}

				//Models
				Curve[gCurveCounter] = CubeMesh->CreateModel(CubeSize * X, 11.0f, CubeSize * Y);
				Curve[gCurveCounter]->SetSkin("RedBall.jpg");
				Curve[gCurveCounter]->Scale(0.1f);
				myEngine->DrawScene();
				gCurveCounter++;
			}
			i++;
		}
		NumLeft = i;
	}

	//Does the remainder of the line
	NumLeft = FinalPath.size() - NumLeft;
	for (int i = FinalPath.size() - NumLeft; i < FinalPath.size() - 1; i++)
	{
		for (int j = 0; j < (CurveDensity / 4) ; j++)
		{
			if (FinalPath[i]->y != FinalPath[i + 1]->y)
			{
				if (FinalPath[i]->y < FinalPath[i + 1]->y)
				{
					X = FinalPath[i]->x;
					Y = FinalPath[i]->y + (j / (fCurve / 4.0f));
				}

				else
				{
					X = FinalPath[i]->x;
					Y = FinalPath[i]->y - (j / (fCurve / 4.0f));
				}
			}

			else
			{
				if (FinalPath[i]->x < FinalPath[i + 1]->x)
				{
					X = FinalPath[i]->x + (j / (fCurve / 4.0f));
					Y = FinalPath[i]->y;
				}

				else
				{
					X = FinalPath[i]->x - (j / (fCurve / 4.0f));
					Y = FinalPath[i]->y;
				}
			}
			//Models
			Curve[gCurveCounter] = CubeMesh->CreateModel(CubeSize * X, 11.0f, CubeSize * Y);
			Curve[gCurveCounter]->SetSkin("RedBall.jpg");
			Curve[gCurveCounter]->Scale(0.1f);
			myEngine->DrawScene();
			gCurveCounter++;
		}
	}
}

//F(n) = G(n) + H(n)
bool AStar(shared_ptr<Node> Start, shared_ptr<Node> Goal, int CurrentMap[gMapWidth][gMapHeight], float myTimer, IModel *PlayerModel, IMesh * CubeMesh, string MapFile)
{
	int SortCount = 0;
	OpenList = { Start };
	shared_ptr<Node> Current(new Node); //Smart Pointer

	while (!OpenList.empty())
	{
		myEngine->DrawScene();
		Current = OpenList[0]; //Takes the front of the list and makes in current
		OpenList.erase(OpenList.begin()); //Get rid of current from open list
		CloseList.push_back(Current); //Add to close list

		//Create Green Model it indicate that we are going this way
		while (myTimer < SearchTime) myTimer = myTimer + myEngine->Timer();
		myTimer = 0;
		cout << "Node found: " << Current->x << ", " << Current->y << endl;


		gModelGreen[Current->x][Current->y] = CubeMesh->CreateModel(CubeSize * Current->x, 11.0f, CubeSize * Current->y);
		PlayerModel->SetPosition(CubeSize * Current->x, 20.0f, CubeSize * Current->y);
		gModelGreen[Current->x][Current->y]->SetSkin("Baize.jpg");
		gModelGreen[Current->x][Current->y]->Scale(0.3f);
		Coords Green = { Current->x, Current->y };
		GreenList.push_back(Green);
		myEngine->DrawScene();

		//Check if we've reached the Goal
		if (Current->x == Goal->x && Current->y == Goal->y)
		{
			reconstructpath(Current, Start, CubeMesh, myTimer, MapFile);
			cout << "Open list sorted " << SortCount << " Times" << endl;
			return true; //Successsss
		}

		//Checks North, East, South, West for new nodes
		if (Current->y < (gMapHeight - 1)) CreateNode(Current->x, Current->y + 1, Goal, Current, CurrentMap, myTimer, CubeMesh);	 //(North) node, 
		if (Current->x < (gMapWidth - 1)) CreateNode(Current->x + 1, Current->y, Goal, Current, CurrentMap, myTimer, CubeMesh);	 //(East) node, 
		if (Current->y > 0) CreateNode(Current->x, Current->y - 1, Goal, Current, CurrentMap, myTimer, CubeMesh);	 //(South) node, 
		if (Current->x > 0) CreateNode(Current->x - 1, Current->y, Goal, Current, CurrentMap, myTimer, CubeMesh);	 //(West) node, 

		//Sort List
		if (!OpenList.empty())
		{
			QuickSort(OpenList, 0, OpenList.size() - 1);
			SortCount++;
		}

	}

	cout << "No Path Found :(" << endl;
	return false; // if no path found
}


bool CheckPointReached(IModel* Player, IModel* CheckPoint, float Radius)
{
	float x = (CheckPoint->GetX() - Player->GetX()) * (CheckPoint->GetX() - Player->GetX());
	float z = (CheckPoint->GetZ() - Player->GetZ()) * (CheckPoint->GetZ() - Player->GetZ());
	float Distance = sqrt(x + z);

	if (Distance - Radius < 0) return true;
	else return false;
}


void Patrol(IModel* Player, int &PatrolNum, float Timer, int  CurrentMap[gMapWidth][gMapHeight])
{
	Timer = myEngine->Timer();
	Player->LookAt(Curve[PatrolNum]);

	//Move at different speeds along terain
	int x = Player->GetX() / 10.0f;
	int y = Player->GetY() / 10.0f;
	kPlayerSpeed = CurrentMap[x][y] * PlayerSpeed;
	Player->MoveLocalZ(PlayerSpeed * Timer);

	//If Reached look at next checkpoint
	if (CheckPointReached(Player, Curve[PatrolNum], PathRadius))
	{
		PatrolNum = PatrolNum + 1;
	}
}


void DeleteEverything(IMesh * CubeMesh, IMesh * SphereMesh, IModel * ModelArray[gMapWidth][gMapHeight])
{
	//Remove Floor Tiles
	for (int i = 0; i < gMapWidth; i++)
	{
		for (int j = 0; j < gMapHeight; j++)
		{
			CubeMesh->RemoveModel(ModelArray[i][j]);
		}
	}

	//Remove red squares
	int i = 0;
	while (i < gCurveCounter)
	{
		SphereMesh->RemoveModel(Curve[i]);
		i++;
	}

	//Remove items from the Lists
	while (!OpenList.empty())
	{
		OpenList.pop_back();
	}

	while (!CloseList.empty())
	{
		CloseList.pop_back();
	}

	myEngine->DrawScene(); //For Good Measure
}

void main()
{
	//Open Tl-Engine Window
	myEngine->StartWindowed(600, 600);
	myEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

	//Variables
	bool Playing = true; //GameState
	int CurrentMap[gMapHeight][gMapWidth]; //Array of all the Map Numbers
	float myTimer = 0; // Timer
	string UserInputMap = ""; // Name of map
	shared_ptr<Node> Start(new Node); 
	shared_ptr<Node> Goal(new Node);

	//Meshes, Models and Camera
	IMesh* CubeMesh = myEngine->LoadMesh("cube.x");
	IMesh* SphereMesh = myEngine->LoadMesh("sphere.x");
	IMesh* DummyMesh = myEngine->LoadMesh("dummy.x");
	IModel * ModelArray[gMapWidth][gMapHeight];
	ICamera * myCamera = myEngine->CreateCamera(kManual, 45.0f, 110.0f, 45.0f);
	myCamera->RotateX(90.0f); // Top down view

	//Game Loop
	while (Playing)
	{
		//Setup Camera
		myCamera->SetPosition(4.50f * gMapWidth , 11.0f * gMapHeight , 4.50f * gMapHeight);
		myCamera->ResetOrientation();
		myCamera->RotateX(90.0f);

		//Get Map from User
		do
		{
			cout << "Please Type the name of the map (xxxMAP.TXT): ";
			cin >> UserInputMap;
		} while (!GetMap(CurrentMap, UserInputMap, Start, Goal)); //Validation Loop

		DisplayMap(CurrentMap);	//Display the loaded Map to console


		////////////////////////
		//Visual Represenation//
		////////////////////////

		//Create grid of floor tiles
		for (int i = 0; i < gMapWidth; i++)
		{
			for (int j = 0; j < gMapHeight; j++)
			{
				ModelArray[i][j] = CubeMesh->CreateModel(CubeSize * i, 0.0f, CubeSize * j);
				switch (CurrentMap[i][j])
				{
				case Wall: ModelArray[i][j]->SetSkin("brick1.jpg");
					break;
				case Clear: ModelArray[i][j]->SetSkin("wood2.jpg");
					break;
				case Wood: ModelArray[i][j]->SetSkin("Grass1.jpg");
					break;
				case Water: ModelArray[i][j]->SetSkin("CueTip.jpg");
					break;
				}
			}
		}

		//Create Player
		IModel *PlayerDummy = DummyMesh->CreateModel(CubeSize * Start->x, 16.0f, CubeSize * Start->y);
		IModel *PlayerModel = SphereMesh->CreateModel(CubeSize * Start->x, 11.0f, CubeSize * Start->y);
		PlayerModel->Scale(0.2f);

		//Create Goal
		IModel *GoalModel = CubeMesh->CreateModel(CubeSize * Goal->x, NodeHeight, CubeSize * Goal->y);
		GoalModel->Scale(0.5f);

		//Start and Goal Validation
		if (CurrentMap[Start->x][Start->y] == 0)
		{
			cout << "the start goal is in a wall... Please adjust your Coords file " << endl;
		}
		else if (CurrentMap[Goal->x][Goal->y] == 0)
		{
			cout << "the end goal is in a wall and cannot be reached... Please Adjust your coords file " << endl;
		}

		else
		{
			// If A route has been found 
			if (AStar(Start, Goal, CurrentMap, myTimer, PlayerModel, SphereMesh, UserInputMap))
			{

				//////////////////
				//Move Character//
				//////////////////

				//set up scene
				PlayerModel->SetPosition(CubeSize * Start->x, 20.0f, CubeSize * Start->y);
				myCamera->SetX(CubeSize * Start->x);
				myCamera->SetZ(CubeSize * Start->y);

				//for (int j = 0; j < 15; j++) //Move Camera Down
				//{
				//	myCamera->MoveY(-4.5f);
				//	myEngine->DrawScene();
				//	while (myTimer < SearchTime / 2.0f) myTimer = myTimer + myEngine->Timer();
				//	myTimer = 0;
				//}

				//Raise Walls
				for (int j = 0; j < 20; j++)
				{
					for (int i = 0; i < gMapWidth; i++)
					{
						for (int j = 0; j < gMapHeight; j++)
						{
							switch (CurrentMap[i][j])
							{
							case Wall:
								ModelArray[i][j]->MoveY(0.5f);
							}
						}
					}
					myEngine->DrawScene();
					while (myTimer < SearchTime / 2.0f) myTimer = myTimer + myEngine->Timer();
					myTimer = 0;
				}

				//Set up the scene to follow the player as it move around the map
				myCamera->SetPosition(0.0f, 41.0f, -5.0f);
				PlayerModel->SetPosition(0.0f, 2.0f, 0.0f);
				myCamera->RotateX(-10.0f);
				myCamera->AttachToParent(PlayerDummy);
				PlayerModel->AttachToParent(PlayerDummy);

				//Follow The Path
				int PatrolRoute = 0;

				while (PatrolRoute < gCurveCounter)
				{
					myEngine->DrawScene();
					Patrol(PlayerDummy, PatrolRoute, PathRadius, CurrentMap);
					PlayerModel->RotateLocalX(PlayerSpeed/10.0f);
				}
			}
		}
		//Ask the user for a new map or to exit the program
		string Answer = "";
		bool Validation = false;

		do
		{
			Validation = false;
			if (Answer != "")
			{
				cout << "Invalid input" << endl;
			}

			cout << "Would you like to try a new map? (y/n): " << endl;
			cin >> Answer;

			if (Answer == "y" || Answer == "n")
			{
				Validation = true;
			}

		} while (!Validation);

		if (Answer == "y") Playing = true;
		else if (Answer == "n")	Playing = false; //Exit Loop And Close the program

		//Delete 
		DummyMesh->RemoveModel(PlayerDummy);
		SphereMesh->RemoveModel(PlayerModel);
		CubeMesh->RemoveModel(GoalModel);
		DeleteEverything(CubeMesh, SphereMesh, ModelArray); //delete floor tiles and red squares
												//Deletes Greens
		while (!GreenList.empty())
		{
			SphereMesh->RemoveModel(gModelGreen[GreenList[0].x][GreenList[0].y]);
			GreenList.erase(GreenList.begin());
		}

		//Deletes reds
		while (!RedList.empty())
		{
			SphereMesh->RemoveModel(gModelRed[RedList[0].x][RedList[0].y]);
			RedList.erase(RedList.begin());
			myEngine->DrawScene();
		}
		gCurveCounter = 0; // Reset 
	}

	// Delete the 3D engine now we are finished with it 	myEngine->DrawScene();
	myEngine->Delete();
	exit(0);
}
