using UnityEngine;
using System.Collections;
using UnityEditor;
public class TSSimpleCarRigger : EditorWindow {
	
	
	class Tire{
		public Vector3 tiresCenter;
		public Bounds bound;
		public GameObject tireObj;
		public string position;
		public int tireGroup;
		public float wRadius;
		public Vector3 size;
		public Bounds localBound;
	}
	Tire[] tire;
	static public GameObject rigCarEmpty;
	//	Transform[] carComponents;
	GameObject newBody;
	Object prueba;
	bool[] guiOptions = {false,false,false,false,false, false, false, false, false, false, false,false, false, false, false};

	int menuStage = 0;

	GameObject center;
	float rotationStep = 45f;
	float positionStep = 0.5f;
	float wheelRStep = 0.01f;
	float frontWRadius = 0f;
	float rearWRadius = 0f;
	Material[] materialsSelected;
	string[] materialsSelectedNames;
	int materialSelected = 0;
	bool drivetrainSelected = false;
	bool convex;
	Vector2 scrollPos;

	Transform blobShadowTransform;
	GameObject steeringIRDSWheel;
	GameObject[] selectedObjs;
	
	
	//not obfuscated
	public bool isTrailer = false;
	public Vector3 size;
	
	public void Init()
	{
		rigCarEmpty = new GameObject();
		rigCarEmpty.name = "Empty Car";
		rigCarEmpty.transform.position = Vector3.zero;
		rigCarEmpty.transform.eulerAngles = Vector3.zero;
		//		carComponents = rigCarEmpty.GetComponentsInChildren<Transform>();
		rigCarEmpty.AddComponent<ITSDirectionReference>();
		
		this.minSize = new Vector2(300,600);
		SceneView.onSceneGUIDelegate += this.OnSceneGUI;
		foreach (SceneView scene in SceneView.sceneViews)
			scene.AlignViewToObject(rigCarEmpty.transform);
	}
	
	void OnDestroy()
	{
		SceneView.onSceneGUIDelegate -= this.OnSceneGUI;
	}
	
	public void OnGUI ()
	{
		scrollPos = EditorGUILayout.BeginScrollView(scrollPos, GUILayout.Width(position.width), GUILayout.Height(position.height));
		GUILayout.BeginHorizontal();
		GUILayout.Space(position.width/2-75f);
		GUILayout.Label("Car Rig - Wizard",GUILayout.Width(150));
		GUILayout.EndHorizontal();
		GUILayout.Space(20);
		GUILayoutOption[] options = new GUILayoutOption[2];
		options[0] = GUILayout.Width(80);
		options[1] = GUILayout.Height(80);
		
		
		//First option for selecting the main doby of the car
		switch(menuStage)
		{
		case 0:

			prueba = EditorGUILayout.ObjectField(prueba, typeof(Object), true);
			if (prueba){
				
				GUILayout.BeginHorizontal();
				if (GUILayout.Button(AssetPreview.GetAssetPreview((GameObject)prueba), options) )
				{
					if (center) DestroyImmediate(center);
					newBody = (GameObject)PrefabUtility.InstantiatePrefab(prueba);
					if (rigCarEmpty.GetComponentsInChildren<Rigidbody>().Length == 0)
					{
						rigCarEmpty.AddComponent<Rigidbody>();
					}
					rigCarEmpty.AddComponent<AudioSource>();
					rigCarEmpty.AddComponent<TSSimpleCar>();
					rigCarEmpty.AddComponent<TSTrafficAI>();
					newBody.transform.position = Vector3.zero;
					newBody.transform.eulerAngles = Vector3.zero;
					carCenter(newBody.transform);
					center.transform.parent = rigCarEmpty.transform;
					center.transform.localPosition = Vector3.zero;
					center.transform.eulerAngles = Vector3.zero;
					size = carSize();
					menuStage = 1;
				}
				GUILayout.Label("<--- Click to continue");
				GUILayout.EndHorizontal();
				GUILayoutOption[] options1 = new GUILayoutOption[2];
				options1[0] = GUILayout.Width (300);
				options1[1] = GUILayout.Height (300);
				GUILayout.Label("Notes:  This rigger is intended for creating \nnew cars, you need to center the car on\nthe three axis (red, blue and green) using\nthe buttons included on this wizard. \nPlease do not manually adjust its\nposition or rotation.", options1);
			}

			break;
		case 1:

			GUILayout.BeginHorizontal();
			GUILayout.Label("Change Position of Car",GUILayout.Width(245));
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			changePosition(center.transform);
			
			// Rotation
			GUILayout.Space(25);
			GUILayout.BeginHorizontal();
			GUILayout.Label("Change Rotation of Car",GUILayout.Width(245));
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			changeRotation(center.transform);

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("Back", GUILayout.Width(50)))
				menuStage = 0;
			GUILayout.Space(15);
			if (GUILayout.Button("Ok", GUILayout.Width(50)))
				menuStage = 2;
			GUILayout.EndHorizontal();

			break;
		case 2:

			GUILayout.Space(20);
			GUILayout.BeginHorizontal();
			GUILayout.Label("Adding Car Wheels \n\n" +
			                "Please select all your Tyre GameObjects \n" +
			                "Exclude the Calippers if the model \nhas them \n\n" +
			                "Press Ok when finished",GUILayout.Width(350));
			GUILayout.EndHorizontal();
			GUILayout.Space(20);
			
			GUILayout.BeginHorizontal();
			
			if (GUILayout.Button("Back", GUILayout.Width(50)))
				menuStage =1;
			
			GUILayout.Space(15);
			
			if (GUILayout.Button("Ok", GUILayout.Width(50)))
			{
				assignWheels();
				SceneView.RepaintAll();
				menuStage = 3;
			}
			GUILayout.EndHorizontal();

			break;
		case 3:

			GUI.changed = false;
			activateWheelGizmos();
			GUILayout.Space(10);
			GUILayout.BeginHorizontal();
			GUILayout.Label("Verify the model's wheels Size",GUILayout.Width(350));
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Front Wheels Radius: ",GUILayout.Width(245));
			options[0] = GUILayout.Width(35);
			options[1] = GUILayout.Height(20);
			GUILayout.EndHorizontal();
			GUILayout.Space(10);
			GUILayout.BeginHorizontal();
			if (GUILayout.Button("+", options) )
			{
				frontWRadius+= wheelRStep;
				changeWheelsRadius(0,frontWRadius);
			}
			
			GUILayout.Space(15);
			if (GUILayout.Button("-", options) )
			{
				frontWRadius-= wheelRStep;
				changeWheelsRadius(0,frontWRadius);
			}
			GUILayout.EndHorizontal();
			
			
			GUILayout.Space(15);
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Aplly Front Wheels Radius to all Wheels? ",GUILayout.Width(245));
			GUILayout.EndHorizontal();
			GUILayout.Space(10);
			
			GUILayout.BeginHorizontal();
			if (GUILayout.Button("Apply", GUILayout.Width(50)) )
			{
				changeWheelsRadius(2,frontWRadius);
			}
			
			GUILayout.EndHorizontal();
			
			
			GUILayout.Space(15);
			
			GUILayout.BeginHorizontal();
			GUILayout.Label("Rear Wheels Radius: ",GUILayout.Width(245));
			
			GUILayout.EndHorizontal();
			GUILayout.Space(10);
			
			
			GUILayout.BeginHorizontal();
			
			if (GUILayout.Button("+", options) )
			{
				rearWRadius+= wheelRStep;
				changeWheelsRadius(1,rearWRadius);
			}
			
			GUILayout.Space(15);
			
			if (GUILayout.Button("-", options) )
			{
				rearWRadius-= wheelRStep;
				changeWheelsRadius(1,rearWRadius);
			}
			GUILayout.EndHorizontal();
			
			GUILayout.Space(20);
			GUILayout.BeginHorizontal();
			GUILayout.Label("Aplly Rear Wheels Radius to all Wheels? ",GUILayout.Width(245));
			
			GUILayout.EndHorizontal();
			
			
			GUILayout.Space(10);
			
			
			GUILayout.BeginHorizontal();
			
			if (GUILayout.Button("Apply", GUILayout.Width(50)) )
			{
				changeWheelsRadius(2,rearWRadius);
			}
			
			GUILayout.EndHorizontal();
			
			
			GUILayout.Space(15);
			GUILayout.BeginHorizontal();
			//				wheelRStep = EditorGUILayout.FloatField("Radius Step: ",Mathf.Clamp(wheelRStep,0f,1f));
			wheelRStep = EditorGUILayout.Slider("Radius Step:",wheelRStep,0f,1f);
			GUILayout.EndHorizontal();
			
			
			GUILayout.BeginHorizontal();
			GUILayout.TextArea("Standard wheel radius for selection in centimeters");
			GUILayout.EndHorizontal();
			
			GUILayout.Space(10);
			
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width/2 - 85);
			if (GUILayout.Button("30", GUILayout.Width(50)))
			{
				rearWRadius = 0.3f;
				frontWRadius = 0.3f;
				changeWheelsRadius(2,rearWRadius);
			}
			
			GUILayout.Space(10);
			if (GUILayout.Button("40", GUILayout.Width(50)))
			{
				rearWRadius = 0.4f;
				frontWRadius = 0.4f;
				changeWheelsRadius(2,rearWRadius);
			}
			
			GUILayout.Space(10);
			if (GUILayout.Button("25", GUILayout.Width(50)))
			{
				rearWRadius = 0.25f;
				frontWRadius = 0.25f;
				changeWheelsRadius(2,rearWRadius);
			}
			
			GUILayout.EndHorizontal();
			GUILayout.Space(20);
			
			drivetrainSelected = true;
			
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width/2 - 75);
			if (GUILayout.Button("Back", GUILayout.Width(50)))
			{
				menuStage = 2;
				drivetrainSelected = false;
			}
			
			GUILayout.Space(20);
			
			if (GUILayout.Button("Continue", GUILayout.Width(80)))
			{
				if(drivetrainSelected || isTrailer)
				{
					deactivateWheelGizmos();
					
					menuStage = 4;
				}else EditorUtility.DisplayDialog("Warning!","Pleas select the desired Drivetrain for this car","Ok");
			}
			GUILayout.EndHorizontal();
			break;

		case 4:

			GUILayout.Space(10);
			GUILayout.BeginHorizontal();
			GUILayout.TextArea("Select the GameObjects " +
			                   "to add a MeshCollider or " +
			                   "BoxCollider to, this would " +
			                   "be the colliders of the car");
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			
			GUILayout.BeginHorizontal();
			
			convex = EditorGUILayout.Toggle("Convex?",convex);
			
			GUILayout.EndHorizontal();
			
			GUILayout.BeginHorizontal();
			
			if (GUILayout.Button("Add MeshCollider", GUILayout.Width(200)))
			{
				if (Selection.activeObject != null)
				{
					foreach(GameObject tempObj in Selection.gameObjects)
					{
						tempObj.AddComponent<MeshCollider>();
						if (convex)
							tempObj.GetComponent<MeshCollider>().convex = true;
					}
				}
			}
			
			GUILayout.EndHorizontal();
			
			GUILayout.Space(20);
			
			GUILayout.BeginHorizontal();
			
			if (GUILayout.Button("Add BoxCollider", GUILayout.Width(200)))
			{
				if (Selection.activeObject != null)
				{
					foreach(GameObject tempObj in Selection.gameObjects)
					{
						tempObj.AddComponent<BoxCollider>();
					}
				}
			}
			
			GUILayout.EndHorizontal();
			
			GUILayout.Space(20);
			
			
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width/2 - 75);
			if (GUILayout.Button("Back", GUILayout.Width(50)))
				menuStage = 3;
			
			GUILayout.Space(20);
			
			if (GUILayout.Button("Continue", GUILayout.Width(80)))
			{
				if(rigCarEmpty.GetComponentInChildren<MeshCollider>() || rigCarEmpty.GetComponentInChildren<BoxCollider>())
				{
					menuStage = 5;
				}else EditorUtility.DisplayDialog("Warning!","Pleas add Colliders for this car","Ok");
			}
			GUILayout.EndHorizontal();

			break;

		case 5:

			GUILayout.Space(10);
			GUILayout.BeginHorizontal();
			GUILayout.TextArea("Select the GameObjects that have the brakelights " +
			                   "and select from the dopdown list " +
			                   "the material used for the brakelights" +
			                   " The shader of this material " +
			                   "would be changed to LightEmmisive");
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			
			if (Selection.activeObject != null)
			{
				//					GameObject[] selectedObjects = Selection.gameObjects;
				if (Selection.activeGameObject.GetComponent<Renderer>())
				{	materialsSelected = Selection.activeGameObject.GetComponent<Renderer>().sharedMaterials;
					materialsSelectedNames = new string[materialsSelected.Length];
					int i = 0;
					foreach(Material material in materialsSelected)
					{
						materialsSelectedNames[i] = material.name;
						i++;
					}
					materialSelected = EditorGUILayout.Popup(materialSelected,materialsSelectedNames);
				}
			}
			
			GUILayout.Space(15);
			
			
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width/2 - 75);
			if (GUILayout.Button("Back", GUILayout.Width(50)))
				menuStage = 4;
			
			GUILayout.Space(20);
			
			if (GUILayout.Button("Continue", GUILayout.Width(80)))
			{
				if(Selection.activeObject)
				{
					menuStage = 6;
					TSSimpleCar simpleCar = rigCarEmpty.GetComponent<TSSimpleCar>();
					int totalbrakeLights = 0;
					foreach (Transform t in Selection.transforms){
						totalbrakeLights += t.GetComponentsInChildren<Renderer>().Length;
					}
					simpleCar.brakeLigths = new Renderer[totalbrakeLights];
					int brakeConuter = 0;
					foreach (Transform t in Selection.transforms){
						foreach(Renderer r in t.GetComponentsInChildren<Renderer>()){
							simpleCar.brakeLigths[brakeConuter] = r;
							brakeConuter++;
						}
					}
					simpleCar.enableDisableBrakeLights = true;
				}else EditorUtility.DisplayDialog("Warning!","Pleas select the desired BrakeLights for this car","Ok");
			}
			GUILayout.EndHorizontal();

			break;

		case 6:

			GUILayout.BeginHorizontal();
			GUILayout.TextArea("Give the car a name to save it as a prefab \n\n" +
			                   "Note: The prefab would be saved under the Resources/Cars Folder");
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			GUILayout.BeginHorizontal();
			rigCarEmpty.name = EditorGUILayout.TextField("Car Name: ",rigCarEmpty.name);
			
			GUILayout.EndHorizontal();
			GUILayout.Space(15);
			
			GUILayout.BeginHorizontal();
			GUILayout.Space(position.width/2 - 75);
			
			rigCarEmpty.GetComponent<Rigidbody>().mass = EditorGUILayout.FloatField("Car Mass: ",rigCarEmpty.GetComponent<Rigidbody>().mass);
			
			if (GUILayout.Button("Save & Finish", GUILayout.Width(150)))
			{
				GameObject CoG = new GameObject("CoG");
				CoG.transform.parent = rigCarEmpty.transform;
				CoG.transform.position = new Vector3(0,0.2f,0);
				TSSimpleCar simpleCar = rigCarEmpty.GetComponent<TSSimpleCar>();
				simpleCar.CoM = CoG.transform;
				simpleCar.EngineTorque = 80;
				simpleCar.brakeTorque = 150;
				simpleCar.maxAcceleration = 5;
				simpleCar.superSimplePhysics = false;
				BoxCollider box = rigCarEmpty.AddComponent<BoxCollider>();
				box.isTrigger = true;
				TSTrafficAI trafficAI = rigCarEmpty.GetComponent<TSTrafficAI>();
				trafficAI.playerSensor = box;
				rigCarEmpty.layer = LayerMask.NameToLayer("Traffic AI");
				rigCarEmpty.tag = "TrafficCar";
				CreatePrefab();

				this.Close();
			}
			GUILayout.EndHorizontal();

			break;

		}
		EditorGUILayout.EndScrollView();
	}
	
	void DestroyChildrens(Transform component)
	{
		foreach (MeshRenderer subComponet in component.GetComponentsInChildren<MeshRenderer>())
			if (!subComponet.name.Contains("BLight"))
				DestroyImmediate(subComponet.gameObject);
		else {
			Transform tempT = subComponet.transform;
			DestroyImmediate(subComponet);
			DestroyImmediate(tempT.GetComponent<MeshFilter>());
			
		}
		
	}
	
	void carCenter(Transform carBody)
	{	
		Bounds bounds;
		Renderer tempRender = carBody.gameObject.GetComponentInChildren<Renderer>();
		bounds = new Bounds (tempRender.bounds.center, Vector3.zero);
		center = new GameObject();
		center.transform.position = Vector3.zero;
		center.transform.eulerAngles = Vector3.zero;
		
		Renderer[] renderers = carBody.GetComponentsInChildren<Renderer> ();
		
		foreach (Renderer renderer in renderers)
		{
			bounds.Encapsulate (renderer.bounds);
		}
		center.transform.position = (bounds.center);
		center.transform.position = new Vector3(center.transform.position.x, center.transform.position.y - bounds.size.y/2,center.transform.position.z);
		carBody.parent = center.transform;
		center.name = "Center";
	}
	
	void findWheelsCenter()
	{	
		for (int w = 0; w < tire.Length; w++)
		{
			Bounds bounds;
			Renderer tempRender = tire[w].tireObj.GetComponentInChildren<Renderer>();
			bounds = new Bounds (tempRender.bounds.center, Vector3.zero);
			for (int r = 0; r < tire.Length; r++)
			{
				if (tire[r].tireGroup == tire[w].tireGroup)
				{
					Renderer[] renderers = tire[r].tireObj.GetComponentsInChildren<Renderer> ();
					foreach (Renderer renderer in renderers)
					{
						bounds.Encapsulate (renderer.bounds);
					}
				}
			}
			tire[w].tiresCenter = bounds.center;
		}
	}
	
	Vector3 findObjectCenter(GameObject Obj)
	{	
		Bounds bounds;
		Renderer tempRender = Obj.GetComponentInChildren<Renderer>();
		bounds = new Bounds (tempRender.bounds.center, Vector3.zero);
		Renderer[] renderers = Obj.GetComponentsInChildren<Renderer> ();
		foreach (Renderer renderer in renderers)
		{
			bounds.Encapsulate (renderer.bounds);
		}
		return bounds.center;
	}
	
	
	
	void findWheelsPosition()
	{
		int y = 0;
		
		GameObject centerOBJ = new GameObject();
		centerOBJ.transform.position = findObjectCenter(rigCarEmpty);
		centerOBJ.transform.eulerAngles = Vector3.zero;
		
		foreach(Tire wheel in tire)
		{ 
			float x = centerOBJ.transform.InverseTransformPoint(wheel.tiresCenter).x;
			float z = centerOBJ.transform.InverseTransformPoint(wheel.tiresCenter).z;
			if (x < 0 && z >0) wheel.position = "FL";
			else if (x > 0 && z >0) wheel.position = "FR";
			else if (x < 0 && z <0) wheel.position = "RL";
			else if (x > 0 && z <0) wheel.position = "RR";
			y++;
		}
		DestroyImmediate(centerOBJ);
	}
	
	WheelCollider[] createAditionalWheels(int wheelCount)
	{
		
		WheelCollider[] wheels = new WheelCollider[wheelCount];
		for (int x = 0; x < wheelCount; x++)
		{
			GameObject copyWheel;
			GameObject tireModel = new GameObject();
			copyWheel = new GameObject();
			copyWheel.transform.parent = rigCarEmpty.transform;
			copyWheel.transform.localPosition = Vector3.zero;
			copyWheel.transform.localEulerAngles = Vector3.zero;
			wheels[x] = copyWheel.AddComponent<WheelCollider>();
			tireModel.transform.parent = copyWheel.transform;
			tireModel.transform.localPosition = Vector3.zero;
			tireModel.transform.localRotation = Quaternion.identity;
			tireModel.name = "Tire";
			tireModel.AddComponent<TSSimpleCar_Wheel>();
			tireModel.GetComponent<TSSimpleCar_Wheel>().CorrespondingCollider = wheels[x];

		}
		return wheels;
	}
	
	

	
	void assignWheels()
	{

		GameObject[] selectedTires =  Selection.gameObjects;
		if (selectedTires.Length > 0)
		{
			tire = new Tire[0];
			
			rigCarEmpty.transform.position = Vector3.zero;
			rigCarEmpty.transform.eulerAngles = Vector3.zero;
			int i = 1;
			foreach (GameObject selectedTire in selectedTires)
			{
				Transform[] subSelectedTires = selectedTire.GetComponentsInChildren<Transform>();
				foreach (Transform tireTransform in subSelectedTires)
				{
					Renderer tireRender = tireTransform.GetComponent<Renderer>();
					if (tireRender)
					{
						System.Array.Resize<Tire>(ref tire,i);
						tire[i-1] = new Tire();
						tire[i-1].tiresCenter = tireRender.bounds.center;
						tire[i-1].size = tireRender.bounds.size;
						tire[i-1].tireObj = tireTransform.gameObject;	
						tire[i-1].wRadius = tireRender.GetComponent<MeshFilter>().sharedMesh.bounds.extents.y;// * tire[i-1].tireObj.transform.localScale.y;
						tire[i-1].localBound = tireRender.GetComponent<MeshFilter>().sharedMesh.bounds;
						tire[i-1].bound = tireRender.bounds;
						i++;
					}
				}
			}
			int tireCount = 1;
			findWheelsPosition();
			System.Array.Sort(tire, tireSizeOrder);
			GameObject[] tireContainer = new GameObject[0];
			for (int r = 0; r < tire.Length; r++)
			{
				if (tire[r].tireGroup != 0) continue;
				tire[r].bound.extents = new Vector3(tire[r].bound.extents.x * 1.5f,tire[r].bound.extents.y *1.5f,tire[r].bound.extents.z *1.5f);
				for (int w = 0; w < tire.Length; w++)
				{
					
					bool conatins = tire[r].bound.Contains(tire[w].bound.min) && tire[r].bound.Contains(tire[w].bound.max) ;//  Mathf.Abs(tire[w].tiresCenter.z) > Mathf.Abs(tire[r].tiresCenter.z) - tire[r].localBound.extents.z &&
					if (tire[r].position == tire[w].position && conatins && r != w)
					{
						tire[r].tireGroup = tireCount;
						tire[w].tireGroup = tireCount;
						System.Array.Resize<GameObject>(ref tireContainer,tireCount);
						
					}else if(tire[r].tireGroup == 0 && w == tire.Length-1)
					{
						tire[r].tireGroup = tireCount;
						System.Array.Resize<GameObject>(ref tireContainer,tireCount);
					}
				}
				tireCount++;
			}
			tireCount--;

			WheelCollider[] tempWheels = createAditionalWheels(tireCount);
			findWheelsCenter();
			int wheelCount = 1;
			TSTrafficAI trafficAI = rigCarEmpty.GetComponent<TSTrafficAI>();
			TSSimpleCar simpleCar = rigCarEmpty.GetComponent<TSSimpleCar>();
			trafficAI.frontWheels = new Transform[0];
			JointSpring spring = new JointSpring();
			spring.spring = 25000;
			spring.damper = 250;
			foreach (WheelCollider wheel in tempWheels)
			{

				wheel.suspensionSpring = spring;
				wheel.suspensionDistance = 0.2f;
				wheel.mass = 5f;
				for (int r = 0; r < tire.Length; r++)
				{
					if (tire[r].tireGroup == wheelCount)
					{
						if (wheel.transform.localPosition == Vector3.zero)wheel.transform.localPosition = tire[r].tiresCenter;
						wheel.name = "Wheel" + tire[r].position + wheelCount;
						wheel.radius = tire[r].wRadius;
						if (tire[r].position == "FR")
							wheel.GetComponentInChildren<TSSimpleCar_Wheel>().WheelPosition = "FR";
						if (tire[r].position == "FL")
							wheel.GetComponentInChildren<TSSimpleCar_Wheel>().WheelPosition = "FL";
						if (tire[r].position == "RL")
							wheel.GetComponentInChildren<TSSimpleCar_Wheel>().WheelPosition = "RL";
						if (tire[r].position == "RR")
							wheel.GetComponentInChildren<TSSimpleCar_Wheel>().WheelPosition = "RR";

						if (tire[r].position == "FR" || tire[r].position == "FL"){

							simpleCar.FrontWheels = simpleCar.FrontWheels.Add(wheel);
							System.Array.Resize<Transform>(ref trafficAI.frontWheels,trafficAI.frontWheels.Length+1);
							trafficAI.frontWheels[trafficAI.frontWheels.Length-1] = wheel.transform;
						}
						if (tire[r].position == "RR"  ||  tire[r].position == "RL"){
							simpleCar.BackWheels = simpleCar.BackWheels.Add(wheel);
						}
						
						foreach(Transform tireT in wheel.GetComponentsInChildren<Transform>())
						{
							
							if (tireT && tireT.name == "Tire")
							{
								tire[r].tireObj.transform.parent = tireT.transform;
								wheel.radius *=tire[r].tireObj.transform.localScale.y;
								if (tire[r].position == "FR" || tire[r].position == "FL") frontWRadius = wheel.radius;
								if (tire[r].position == "RR" || tire[r].position == "RL") rearWRadius = wheel.radius;
							}
						}
					}
					
				}
				wheelCount++;
			}
			addAntirollbars();
			Selection.activeObject =  null;
		}else{
			guiOptions[2] = false;
			EditorUtility.DisplayDialog("Warning!","Please select at least 3 GameObject which represent the model's Tyres!","Ok");
		}
	}
	

	
	void changeWheelsRadius(int pos, float radius)
	{
		WheelCollider[] wheels = rigCarEmpty.GetComponentsInChildren<WheelCollider>();
		
		foreach(WheelCollider wheel in wheels)
		{
			switch(pos){
			case 0:
				if (wheel.name.Contains("FR") || wheel.name.Contains( "FL"))
					wheel.radius = radius;
				break;
			case 1:
				if (wheel.name.Contains("RR") || wheel.name.Contains("RL"))
					wheel.radius = radius;
				break;
			case 2:
				wheel.radius = radius;
				break;
				
			}
		}
		SceneView.lastActiveSceneView.Repaint();
		
	}
	
//	void addAntirollbars()
//	{
//		TSSimpleCar_Wheel[] tempWheels = rigCarEmpty.GetComponentsInChildren<TSSimpleCar_Wheel>();
//		
//		TSAntiRollBar[] tempAntiRollbars = rigCarEmpty.GetComponentsInChildren<TSAntiRollBar>();
//		
//		foreach(TSAntiRollBar rollbar in tempAntiRollbars)
//			DestroyImmediate(rollbar);
//		
//		for (int i = 0; i < tempWheels.Length / 2 ; i++)
//			rigCarEmpty.AddComponent<TSAntiRollBar>();
//		
//		tempAntiRollbars = rigCarEmpty.GetComponentsInChildren<TSAntiRollBar>();
//		
//		for(int i = 0; i < tempAntiRollbars.Length;i++)
//		{
//			foreach(TSSimpleCar_Wheel wheel in tempWheels)
//			{
//				foreach(TSSimpleCar_Wheel wheel1 in tempWheels)
//				{
//					bool can = true;
//					for (int r = 0; r < i; r++)
//					{
//						if ((tempAntiRollbars[r].wheel1.transform.parent.name == wheel.transform.parent.name && tempAntiRollbars[r].wheel2.transform.parent.name == wheel1.transform.parent.name) ||(tempAntiRollbars[r].wheel1.transform.parent.name == wheel1.transform.parent.name && tempAntiRollbars[r].wheel2.transform.parent.name == wheel.transform.parent.name))
//							can = false;
//					}
//					if (can && (tempAntiRollbars[i].wheel1 == null && tempAntiRollbars[i].wheel2 == null) &&(wheel.WheelPosition != wheel1.WheelPosition && wheel.WheelPosition.Substring(0,1) == wheel1.WheelPosition.Substring(0,1)) && Mathf.Abs(rigCarEmpty.transform.InverseTransformPoint(wheel.transform.position).magnitude - rigCarEmpty.transform.InverseTransformPoint( wheel1.transform.position).magnitude) < 0.1f)
//					{
//						w++;
//						tempAntiRollbars[i].wheel1 = wheel;
//						tempAntiRollbars[i].wheel2 = wheel1;
//					}
//				}
//			}
//		}
//	}
	void addAntirollbars()
	{
		TSSimpleCar_Wheel[] tempWheels = rigCarEmpty.GetComponentsInChildren<TSSimpleCar_Wheel>();
		
		TSAntiRollBar[] tempAntiRollbars = rigCarEmpty.GetComponentsInChildren<TSAntiRollBar>();
		
		foreach (TSAntiRollBar rollbar in tempAntiRollbars)
			DestroyImmediate(rollbar);
		
		for (int i = 0; i < tempWheels.Length / 2; i++)
			rigCarEmpty.AddComponent<TSAntiRollBar>();
		
		tempAntiRollbars = rigCarEmpty.GetComponentsInChildren<TSAntiRollBar>();
		
		for (int i = 0; i < tempAntiRollbars.Length; i++)
		{
			foreach (TSSimpleCar_Wheel wheel in tempWheels)
			{
				foreach (TSSimpleCar_Wheel wheel1 in tempWheels)
				{
					bool can = true;
					for (int r = 0; r < i; r++)
					{
						if (tempAntiRollbars[r].wheel1 != null && tempAntiRollbars[r].wheel2 != null)
							if ((tempAntiRollbars[r].wheel1.transform.parent.name == wheel.transform.parent.name && tempAntiRollbars[r].wheel2.transform.parent.name == wheel1.transform.parent.name) || (tempAntiRollbars[r].wheel1.transform.parent.name == wheel1.transform.parent.name && tempAntiRollbars[r].wheel2.transform.parent.name == wheel.transform.parent.name))
								can = false;
					}
					//if (can && (tempAntiRollbars[i].wheel1 == null && tempAntiRollbars[i].wheel2 == null) && (wheel.WheelPosition != wheel1.WheelPosition && wheel.WheelPosition.Substring(0, 1) == wheel1.WheelPosition.Substring(0, 1)) && Mathf.Abs(rigCarEmpty.transform.InverseTransformPoint(wheel.transform.position).magnitude - rigCarEmpty.transform.InverseTransformPoint(wheel1.transform.position).magnitude) < 0.1f)
					if (can && (tempAntiRollbars[i].wheel1 == null && tempAntiRollbars[i].wheel2 == null) &&(wheel.WheelPosition != wheel1.WheelPosition && wheel.WheelPosition.Substring(0,1) == wheel1.WheelPosition.Substring(0,1)) && Mathf.Abs(rigCarEmpty.transform.InverseTransformPoint(wheel.transform.position).z - rigCarEmpty.transform.InverseTransformPoint( wheel1.transform.position).z) < 0.01f)
					{
						w++;
						tempAntiRollbars[i].wheel1 = wheel;
						tempAntiRollbars[i].wheel2 = wheel1;
					}
				}
			}
		}
	}
	
	int tireSizeOrder ( Tire x, Tire y)
	{
		float magnitude1 = -x.localBound.size.magnitude;
		float magnitude2 = -y.localBound.size.magnitude;
		return magnitude1.CompareTo( magnitude2 );
	}
	

	
	void activateWheelGizmos()
	{
		WheelCollider[] tempWheels = rigCarEmpty.GetComponentsInChildren<WheelCollider>();
		if (!tempWheels[0].GetComponent<ITSWheelSize>())
			foreach(WheelCollider wheel in tempWheels)
		{
			wheel.gameObject.AddComponent<ITSWheelSize>();
		}
	}
	
	void deactivateWheelGizmos()
	{
		WheelCollider[] tempWheels = rigCarEmpty.GetComponentsInChildren<WheelCollider>();
		if (tempWheels[0].GetComponent<ITSWheelSize>())
			foreach(WheelCollider wheel in tempWheels)
		{
			DestroyImmediate(wheel.gameObject.GetComponent<ITSWheelSize>());
		}
	}
	
	 
	
	
	private float w = 0;
	
	void OnSceneGUI(SceneView sceneView) 
	{
		if (rigCarEmpty!=null){


			GUIStyle newStyle = new GUIStyle();
			newStyle.fontSize = 20;
			newStyle.fontStyle = FontStyle.Bold;
			newStyle.normal.textColor = Color.yellow;
			
			Handles.Label(rigCarEmpty.transform.position+ rigCarEmpty.transform.right * 3 - rigCarEmpty.transform.up * 1.0f,"Car total Length:  "+size.z, newStyle);
			Handles.Label(rigCarEmpty.transform.position+ rigCarEmpty.transform.right * 3 - rigCarEmpty.transform.up * 1.5f,"Car total Width:   "+size.x, newStyle);
			Handles.Label(rigCarEmpty.transform.position+ rigCarEmpty.transform.right * 3 - rigCarEmpty.transform.up * 2.0f,"Car total Height:  "+size.y, newStyle);
			
			newStyle.normal.textColor = Color.white;
			newStyle.fontSize = 15;
			Handles.Label(rigCarEmpty.transform.position+ rigCarEmpty.transform.right * 2,"0mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 1+ rigCarEmpty.transform.right * 2,"1Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 2+ rigCarEmpty.transform.right * 2,"2Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 3+ rigCarEmpty.transform.right * 2,"3Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 4+ rigCarEmpty.transform.right * 2,"4Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 5+ rigCarEmpty.transform.right * 2,"5Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position - rigCarEmpty.transform.forward * 6+ rigCarEmpty.transform.right * 2,"6Mts", newStyle);
			
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 1+ rigCarEmpty.transform.right * 2,"1Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 2+ rigCarEmpty.transform.right * 2,"2Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 3+ rigCarEmpty.transform.right * 2,"3Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 4+ rigCarEmpty.transform.right * 2,"4Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 5+ rigCarEmpty.transform.right * 2,"5Mts", newStyle);
			Handles.Label(rigCarEmpty.transform.position + rigCarEmpty.transform.forward * 6+ rigCarEmpty.transform.right * 2,"6Mts", newStyle);
		}
	}
	
	
	
	
	static void CreatePrefab()
	{
		string Path1 = "Assets/iTS/Vehicles/";
		string name = rigCarEmpty.name;
		string localPath = Path1 + name + ".prefab";
		
		if (AssetDatabase.LoadAssetAtPath(localPath, typeof(GameObject)))
		{
			if (EditorUtility.DisplayDialog("Are you sure?", "The prefab already exists. Do you want to overwrite it?", "Yes", "No"))
			{
				if (rigCarEmpty.GetComponent<ITSDirectionReference>())
					DestroyImmediate(rigCarEmpty.GetComponent<ITSDirectionReference>());
				createNew(rigCarEmpty, localPath);
			}
		}
		else
		{
			if (rigCarEmpty.GetComponent<ITSDirectionReference>())
				DestroyImmediate(rigCarEmpty.GetComponent<ITSDirectionReference>());
			createNew(rigCarEmpty, localPath);
		}
		
	}
	
	
	static void createNew(GameObject obj, string localPath)
	{
		Object prefab = PrefabUtility.CreatePrefab (localPath, obj);
		PrefabUtility.ReplacePrefab(obj, prefab, ReplacePrefabOptions.ConnectToPrefab);
		AssetDatabase.Refresh();
	}
	
	
	

	void changeRotation(Transform objectToRotate)
	{
		// Rotation
		GUILayoutOption[] options = new GUILayoutOption[2];
		options[0] = GUILayout.Width(35);
		options[1] = GUILayout.Height(20);
		GUILayout.Space(15);
		
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X+", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x + rotationStep ,
			                                         objectToRotate.eulerAngles.y ,objectToRotate.eulerAngles.z);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y+", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x  ,
			                                         objectToRotate.eulerAngles.y + rotationStep ,objectToRotate.eulerAngles.z);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z+", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x  ,
			                                         objectToRotate.eulerAngles.y  ,objectToRotate.eulerAngles.z + rotationStep);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		
		GUILayout.EndHorizontal();
		GUILayout.Space(15);
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X-", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x  - rotationStep  ,
			                                         objectToRotate.eulerAngles.y  ,objectToRotate.eulerAngles.z);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y-", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x   ,
			                                         objectToRotate.eulerAngles.y - rotationStep  ,objectToRotate.eulerAngles.z);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z-", options) ){
			objectToRotate.eulerAngles = new Vector3(objectToRotate.eulerAngles.x   ,
			                                         objectToRotate.eulerAngles.y  ,objectToRotate.eulerAngles.z - rotationStep);
			Selection.activeGameObject = objectToRotate.gameObject;
		}
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		//		rotationStep = EditorGUILayout.FloatField("Rotation Step: ",Mathf.Clamp(rotationStep,0f,360f));
		rotationStep = EditorGUILayout.Slider("Rotation Step:",rotationStep,0f,360f);
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		if (GUILayout.Button("Reset Rotation", GUILayout.Width(150)) )
			objectToRotate.eulerAngles = Vector3.zero;
		
		GUILayout.EndHorizontal();
	}
	
	
	
	void changePosition(Transform Obj)
	{
		GUILayoutOption[] options = new GUILayoutOption[2];
		options[0] = GUILayout.Width(35);
		options[1] = GUILayout.Height(20);
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X+", options) ){
			Obj.position = new Vector3(Obj.position.x + positionStep,Obj.position.y,
			                           Obj.position.z);
			Selection.activeGameObject = Obj.gameObject;}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y+", options) ){
			Obj.position = new Vector3(Obj.position.x,
			                           Obj.position.y + positionStep,Obj.position.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z+", options) ){
			Obj.position = new Vector3(Obj.position.x,
			                           Obj.position.y,Obj.position.z + positionStep);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.EndHorizontal();
		GUILayout.Space(15);
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X-", options) ){
			Obj.position = new Vector3(Obj.position.x  - positionStep,Obj.position.y,
			                           Obj.position.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y-", options) ){
			Obj.position = new Vector3(Obj.position.x ,
			                           Obj.position.y - positionStep,Obj.position.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z-", options) ){
			Obj.position = new Vector3(Obj.position.x ,
			                           Obj.position.y ,Obj.position.z- positionStep);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		//		positionStep = EditorGUILayout.FloatField("Position Step: ",Mathf.Clamp(positionStep,0f,10f));
		positionStep = EditorGUILayout.Slider("Position Step:",positionStep,0f,10f);
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		if (GUILayout.Button("Reset Position", GUILayout.Width(150)) )
		{
			Obj.position = Vector3.zero;
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.EndHorizontal();
	}
	
	
	void changeScale(Transform Obj)
	{
		GUILayoutOption[] options = new GUILayoutOption[2];
		options[0] = GUILayout.Width(35);
		options[1] = GUILayout.Height(20);
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X+", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x + positionStep ,
			                             Obj.localScale.y ,Obj.localScale.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y+", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x  ,
			                             Obj.localScale.y + positionStep ,Obj.localScale.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z+", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x  ,
			                             Obj.localScale.y  ,Obj.localScale.z + positionStep);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		
		GUILayout.EndHorizontal();
		GUILayout.Space(15);
		GUILayout.BeginHorizontal();
		if (GUILayout.Button("X-", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x  - positionStep  ,
			                             Obj.localScale.y  ,Obj.localScale.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Y-", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x   ,
			                             Obj.localScale.y - positionStep  ,Obj.localScale.z);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.Space(15);
		if (GUILayout.Button("Z-", options) ){
			Obj.localScale = new Vector3(Obj.localScale.x   ,
			                             Obj.localScale.y  ,Obj.localScale.z - positionStep);
			Selection.activeGameObject = Obj.gameObject;
		}
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		//		positionStep = EditorGUILayout.FloatField("Scale/Position Step: ",Mathf.Clamp(positionStep,0f,10f));
		positionStep = EditorGUILayout.Slider("Scale/Position Step:",positionStep,0f,10f);
		
		GUILayout.EndHorizontal();
		
		GUILayout.BeginHorizontal();
		
		if (GUILayout.Button("Reset Scale", GUILayout.Width(150)) )
			Obj.localScale = Vector3.one;
		
		GUILayout.EndHorizontal();
	}
	
	Vector3 carSize()
	{	
		Bounds bounds;
		Quaternion temprotation = rigCarEmpty.transform.rotation;
		Vector3 tempPosition=rigCarEmpty.transform.position;
		rigCarEmpty.transform.rotation = Quaternion.Euler(Vector3.zero);
		rigCarEmpty.transform.position = Vector3.zero;
		bounds = new Bounds (rigCarEmpty.transform.position, Vector3.zero);
		Renderer[] renderers = rigCarEmpty.GetComponentsInChildren<Renderer> ();
		
		foreach (Renderer renderer in renderers)
			
		{
			if (renderer.sharedMaterial && renderer.sharedMaterial.shader.name != "AngryBots/FX/Multiply")
				bounds.Encapsulate (renderer.bounds);
		}
		
		rigCarEmpty.transform.rotation = temprotation;
		rigCarEmpty.transform.position = tempPosition;
		return new Vector3(bounds.size.x,bounds.size.y,bounds.size.z);
	}
	
}//end of class
