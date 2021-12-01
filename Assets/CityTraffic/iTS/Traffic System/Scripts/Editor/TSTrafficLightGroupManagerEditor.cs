using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;


[CustomEditor(typeof(TSTrafficLightGroupManager))]
public class TSTrafficLightGroupManagerEditor : Editor {

	TSTrafficLightGroupManager manager;
	TSTrafficLight[] tlights;
	Color defaultColor;
	bool hideDetails = true;
	Texture2D tex = new Texture2D(1,1);
	float[] totalTime = new float[0];
	bool ligthsTimeNotSync = false;




	[MenuItem("GameObject/iTS/Traffic Lights/Add traffic light script to selected")]
	static void AddTrafficLight () {
		GameObject[] selections = Selection.gameObjects;
		List<string> result = new List<string>();
		if (selections.Length !=0)
		{
			for (int i = 0; i < selections.Length; i++)
			{
				if (selections[i].GetComponent<TSTrafficLight>() == null) {
					selections[i].AddComponent<TSTrafficLight>();
				}else{
					result.Add(selections[i].name);
				}
			}
		}
		else
		{
			EditorUtility.DisplayDialog("Warning!", "One or more objects needs to be selected in order to add the traffic light script to them!","Ok");
		}
		if (result.Count !=0){
			string objects = "";
			for (int i =0; i < result.Count;i++)
			{
				if (i != result.Count-1)
					objects += result[i] + ", ";
				else objects += result[i];
			}
			EditorUtility.DisplayDialog("Warning!", "The following objects " + objects + " already have the TSTrafficLight script on them!","Ok");
		}
	}


	[MenuItem("GameObject/iTS/Traffic Lights/Group")]
	static void CreateTrafficLightsGroup () {
		GameObject[] selections = Selection.gameObjects;
		bool cantAdd = false;
		for (int i = 0; i < selections.Length; i++)
		{
			if (selections[i].GetComponent<TSTrafficLight>() == null) {
				cantAdd = true;
				break;
			}
		}
		if (cantAdd) {
			EditorUtility.DisplayDialog("Warning!", "One or more objects are not traffic lights, or doesn't have the TSTrafficLight script assigned to them, please only select traffic lights to make the group!","Ok");
			return;
		}
		GameObject newGroup = new GameObject();
		newGroup.name = "New Traffic lights group";
		Vector3 tempPos = Vector3.zero;
		for (int i = 0; i < selections.Length; i++)
		{
			tempPos += selections[i].transform.position;
		}
		tempPos /=selections.Length;
		newGroup.transform.position = tempPos;
		for (int i = 0; i < selections.Length; i++)
		{
			selections[i].transform.parent = newGroup.transform;
		}
		newGroup.AddComponent<TSTrafficLightGroupManager>();
	}



	public void OnEnable() 
	{ 
		defaultColor =GUI.color;
		manager = (TSTrafficLightGroupManager)target;
		tlights = manager.GetComponentsInChildren<TSTrafficLight>();
		UpdateLightsTimes();
	}

	void UpdateLightsTimes()
	{
		totalTime = new float[tlights.Length]; 
		float lastTime = 0;
		ligthsTimeNotSync = false;
		for (int i =0;i<tlights.Length;i++)
		{
			totalTime[i] = 0;
			for (int w = 0; w < tlights[i].lights.Count;w++)
			{
				totalTime[i]+= tlights[i].lights[w].lightTime;
			}
			if (i>0 && lastTime != totalTime[i])ligthsTimeNotSync = true;
			lastTime = totalTime[i];
		}
	}

	void SetDirtyLights()
	{
		for (int i =0;i<tlights.Length;i++)
		{
			EditorUtility.SetDirty(tlights[i]);
		}
	}

	public override void OnInspectorGUI()
	{
		GUILayout.Space(25);
		EditorGUILayout.BeginVertical(EditorStyles.textField);
		manager.greenLightTime = EditorGUILayout.FloatField("Green light time:",manager.greenLightTime);
		manager.yellowLightTime = EditorGUILayout.FloatField("Yellow light time:",manager.yellowLightTime);

		if (GUILayout.Button("Auto Sync Traffic Lights"))
		{
			int result = EditorUtility.DisplayDialogComplex("Warning!","The traffic lights would be arrenged with the light times defined above, and would reset the actual timming settings!\nProceed?","Yes","No","Cancel");
			if (result ==0)
			{
				AutoSyncTrafficLights();
			}
		}

		EditorGUILayout.EndVertical();

		GUILayout.Space(25);
		int i = 0;
		hideDetails = !GUILayout.Toggle(!hideDetails, "Details", EditorStyles.miniButton,GUILayout.Width(100));      

		foreach(TSTrafficLight tLight in tlights)
		{
			GUI.color = defaultColor;
			GUILayout.BeginVertical(EditorStyles.textField);
			LightsOptions(tLight, i);
			GUILayout.Space(5);
			GUILayout.EndVertical();
			GUILayout.Space(5);
			i++;
		}

		GUILayout.Space(5);

		if (ligthsTimeNotSync)EditorGUILayout.HelpBox("One or more traffic lights have different total time, this would cause the traffic ligths to be out of sync!", MessageType.Warning);

		GUILayout.Space(5);
		GUILayout.Label("Traffic lights timeline");

		Rect tempRect1 = GUILayoutUtility.GetRect(1,15);
		float width1 =128;
		int minutes = 0;
		int seconds = 0;
		for (int i1 = 0; i1< 20;i1++)
		{
			if (seconds >= 60){ seconds = 0;
				minutes++;
			}
			GUI.Label(new Rect(tempRect1.x+width1,tempRect1.y,50,15), minutes + ":" + (seconds==0?"00":seconds.ToString()),EditorStyles.miniTextField);
			width1 +=50;
			seconds+=10;
		}
		foreach(TSTrafficLight tLight in tlights)
		{
			GUI.color = defaultColor;
			GUILayout.BeginVertical(EditorStyles.textField);
			GUILayout.BeginHorizontal();
			GUILayout.Label(tLight.name, GUILayout.Width(120));
			Rect tempRect = GUILayoutUtility.GetRect(1,15);
			float width = 0;
			for (int ii =0; ii < tLight.lights.Count;ii++)
			{
				
				switch(tLight.lights[ii].lightType)
				{
				case TSTrafficLight.LightType.Yellow:
					GUI.color = Color.yellow;
					break;
				case TSTrafficLight.LightType.Red:
					GUI.color = Color.red;
					break;
				case TSTrafficLight.LightType.Green:
					GUI.color = Color.green;
					break;
				}
				
				GUI.Label(new Rect(tempRect.x+width,tempRect.y,tLight.lights[ii].lightTime*5,15), tex,EditorStyles.miniTextField);
				width +=tLight.lights[ii].lightTime*5;
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			i++;
		}

		if (GUI.changed)
		{
			EditorUtility.SetDirty(target);
			SetDirtyLights();
			UpdateLightsTimes();
		}

	}

	void AddeNewLight(ref TSTrafficLight tLight)
	{
		tLight.lights.Add(new TSTrafficLight.TSLight());
		tLight.lights[tLight.lights.Count-1].lightTime = 15;
		tLight.lights[tLight.lights.Count-1].shaderTexturePropertyName = "_MainTex";
	}

	void LightsOptions(TSTrafficLight tLight, int ii)
	{
		GUILayout.BeginHorizontal(EditorStyles.textField);
		GUILayout.BeginHorizontal(GUILayout.Width(250));
		GUILayout.Space(5);
		GUILayout.BeginVertical(GUILayout.Width(100));
		if (!hideDetails){

			if (GUILayout.Button("Add new light"))
			{
				AddeNewLight(ref tLight);
			}

			GUILayout.Space(70);

		}
		GUILayout.Label("Traffic Light: " + tLight.name, GUILayout.Width(75 + tLight.name.Length * 6));
		GUILayout.BeginHorizontal();
		if (totalTime != null)
			GUILayout.Label("Total time: " + totalTime[ii].ToString());
		else UpdateLightsTimes();
		if (GUILayout.Button("Add Light"))
			AddeNewLight(ref tLight);
		GUILayout.EndHorizontal();
		GUILayout.EndVertical ();
		GUILayout.EndHorizontal();
		GUILayout.Space(5);



		for (int i = 0; i < tLight.lights.Count; i++)
		{

			if ((Application.isPlaying && tLight.lightToPlay == i)|| (Application.isEditor && !Application.isPlaying)){
				int index =tLight.lightToPlay;
				if (Application.isEditor  && !Application.isPlaying) index = i;
				switch(tLight.lights[index].lightType)
				{
				case TSTrafficLight.LightType.Yellow:
					GUI.color = Color.yellow;
					break;
				case TSTrafficLight.LightType.Red:
					GUI.color = Color.red;
					break;
				case TSTrafficLight.LightType.Green:
					GUI.color = Color.green;
					break;
				}
			}
			else GUI.color = defaultColor;



			GUILayout.BeginVertical(EditorStyles.textField, GUILayout.Width(60));
			GUI.color = defaultColor;

			GUILayout.BeginVertical(GUILayout.Width(30));

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("<", EditorStyles.miniButton, GUILayout.Width(20)) && i !=0){
				Move<TSTrafficLight.TSLight>(tLight.lights,i,i-1);
				
			}
			if (GUILayout.Button(">", EditorStyles.miniButton, GUILayout.Width(20))){
				Move<TSTrafficLight.TSLight>(tLight.lights,i,i+1);
			}

			if (GUILayout.Button("L"))
			{
				tLight.lights[i].lightType++;
				if (tLight.lights[i].lightType > TSTrafficLight.LightType.NoLights)
					tLight.lights[i].lightType = (TSTrafficLight.LightType)0;
			}

			GUILayout.EndHorizontal();

			if (hideDetails)
			{
				GUILayout.BeginHorizontal();
				GUILayout.Label("T",GUILayout.Width(10));
				tLight.lights[i].lightTime = EditorGUILayout.FloatField(tLight.lights[i].lightTime, GUILayout.Width(35));
				GUILayout.EndHorizontal();
			}

			GUILayout.EndVertical();
			if (!hideDetails){
				GUILayout.Label("Light "+tLight.lights[i].lightType + " " + i);
				if (GUILayout.Button("Delete", EditorStyles.miniButton, GUILayout.Width(80))){
					tLight.lights.Remove(tLight.lights[i]);
					break;
				}

				GUILayout.BeginHorizontal();
				GUILayout.Label("Light Type: ");
				tLight.lights[i].lightType = (TSTrafficLight.LightType) EditorGUILayout.EnumPopup(tLight.lights[i].lightType, GUILayout.Width(100));
				GUILayout.EndHorizontal();

				GUILayout.BeginHorizontal();
				GUILayout.Label("Mesh Renderer");
				tLight.lights[i].lightMeshRenderer = (MeshRenderer)EditorGUILayout.ObjectField(tLight.lights[i].lightMeshRenderer, typeof(MeshRenderer),true, GUILayout.Width(100));
				GUILayout.EndHorizontal();

				GUILayout.BeginHorizontal();
				GUILayout.Label("Enable / Disable Renderer?");
				tLight.lights[i].enableDisableRenderer = EditorGUILayout.Toggle(tLight.lights[i].enableDisableRenderer, GUILayout.Width(100));
				GUILayout.EndHorizontal();

				GUILayout.BeginHorizontal();
				GUILayout.Label("Optional GameObject");
				tLight.lights[i].lightGameObject = (GameObject)EditorGUILayout.ObjectField(tLight.lights[i].lightGameObject, typeof(GameObject),true);
				GUILayout.EndHorizontal();

				GUILayout.BeginHorizontal();
				GUILayout.Label("Texture");
				tLight.lights[i].lightTexture = (Texture2D)EditorGUILayout.ObjectField(tLight.lights[i].lightTexture, typeof(Texture2D),false , GUILayout.Width(100));
				GUILayout.EndHorizontal();

				GUILayout.BeginHorizontal();
				GUILayout.Label("Shader Texture name");
				tLight.lights[i].shaderTexturePropertyName = EditorGUILayout.TextField(tLight.lights[i].shaderTexturePropertyName, GUILayout.Width(100));
				GUILayout.EndHorizontal();
		
				GUILayout.BeginHorizontal();
				GUILayout.Label("Time");
				tLight.lights[i].lightTime = EditorGUILayout.FloatField(tLight.lights[i].lightTime, GUILayout.Width(100));
				GUILayout.EndHorizontal();
			}
			GUILayout.Space(5);
			GUILayout.EndVertical();
			GUILayout.Space(5);
		}
		GUILayout.EndHorizontal();
	}

	void Move<T>(List<T> list, int oldIndex, int newIndex)
	{
		T aux = list[newIndex];
		list[newIndex] = list[oldIndex];
		list[oldIndex] = aux;
	}

	/// <summary>
	/// Auto syncs traffic lights.
	/// </summary>
	void AutoSyncTrafficLights()
	{
		int t = 0;
		int totalAmountOfLights = tlights.Length;
		foreach(TSTrafficLight tLight in tlights)
		{
			if (t ==0)
			{
				//First traffic light
				CheckAndCreateMissingLights(tLight,false);
				AssignLightsTimes(tLight, true, false, totalAmountOfLights,t);
			}else if (t < tlights.Length-1)
			{
				//Middle traffic lights
				CheckAndCreateMissingLights(tLight,true);
				AssignLightsTimes(tLight, false, true, totalAmountOfLights,t);

			}else{
				//Final traffic light
				CheckAndCreateMissingLights(tLight,false);
				AssignLightsTimes(tLight, false, false, totalAmountOfLights,t);
			}
			t++;

		}

	}


	/// <summary>
	/// Checks the and create missing lights.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="inBetween">If set to <c>true</c> in between.</param>
	void AssignLightsTimes(TSTrafficLight tLight,bool first, bool inBetween, int totalLights, int currentLight){


		if (first)
		{
			AssignFirstLightTimes(tLight,totalLights);

		}else if (inBetween)
		{
			AssignMiddleLightTimes(tLight,totalLights,currentLight);
		}
		else if (!first)
		{
			AssignLastLightTimes(tLight,totalLights);
		}
	}

	/// <summary>
	/// Assigns the first light times.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="totalLights">Total lights.</param>
	void AssignFirstLightTimes(TSTrafficLight tLight, int totalLights)
	{
		bool arranged = false;
		int t = 0;
		while(!arranged)
		{
			for (int i =0;i < tLight.lights.Count;i++)
			{
				switch(t)
				{
					//green light
				case 0:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Green)
					{
						tLight.lights[i].lightTime = manager.greenLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,0);
						t = 1;
					}
					break;
					//yellow light
				case 1:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Yellow)
					{
						tLight.lights[i].lightTime = manager.yellowLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,1);
						t = 2;
					}
					break;
					//red light
				case 2:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Red)
					{
						tLight.lights[i].lightTime = (manager.greenLightTime + manager.yellowLightTime) * (totalLights-1);
						Move<TSTrafficLight.TSLight>(tLight.lights,i,2);
						t = 3;
					}
					
					break;
				}
			}
			if (t == 3) arranged = true;
		}
	}

	/// <summary>
	/// Assigns the last light times.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="totalLights">Total lights.</param>
	void AssignLastLightTimes(TSTrafficLight tLight, int totalLights)
	{
		bool arranged = false;
		int t = 0;
		while(!arranged)
		{
			for (int i =0;i < tLight.lights.Count;i++)
			{
				switch(t)
				{
				case 0:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Red)
					{
						tLight.lights[i].lightTime = (manager.greenLightTime + manager.yellowLightTime) * (totalLights-1);
						Move<TSTrafficLight.TSLight>(tLight.lights,i,0);
						t = 1;
					}
					break;
					//green light
				case 1:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Green)
					{
						tLight.lights[i].lightTime = manager.greenLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,1);
						t = 2;
					}
					break;
					//yellow light
				case 2:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Yellow)
					{
						tLight.lights[i].lightTime = manager.yellowLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,2);
						t = 3;
					}
					break;
				}
			}
			if (t == 3) arranged = true;
		}
	}

	/// <summary>
	/// Assigns the middle light times.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="totalLights">Total lights.</param>
	/// <param name="currentLight">Current light.</param>
	void AssignMiddleLightTimes(TSTrafficLight tLight, int totalLights, int currentLight)
	{
		bool arranged = false;
		int t = 0;
		while(!arranged)
		{
			for (int i =0;i < tLight.lights.Count;i++)
			{
				switch(t)
				{
					//red light
				case 0:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Red)
					{
						tLight.lights[i].lightTime = (manager.greenLightTime + manager.yellowLightTime) * (currentLight);
						Move<TSTrafficLight.TSLight>(tLight.lights,i,0);
						t = 1;
					}
					
					break;
					//green light
				case 1:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Green)
					{
						tLight.lights[i].lightTime = manager.greenLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,1);
						t = 2;
					}
					break;
					//yellow light
				case 2:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Yellow)
					{
						tLight.lights[i].lightTime = manager.yellowLightTime;
						Move<TSTrafficLight.TSLight>(tLight.lights,i,2);
						t = 3;
					}
					break;
					//red light
				case 3:
					if (tLight.lights[i].lightType == TSTrafficLight.LightType.Red)
					{
						tLight.lights[i].lightTime = (manager.greenLightTime + manager.yellowLightTime) * (totalLights-1-currentLight);
						Move<TSTrafficLight.TSLight>(tLight.lights,i,3);
						t = 4;
					}
					
					break;
				}
			}
			if (t == 4) arranged = true;
		}
	}



	/// <summary>
	/// Checks the and create missing lights.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="inBetween">If set to <c>true</c> in between.</param>
	void CheckAndCreateMissingLights(TSTrafficLight tLight, bool inBetween){
		int numberOfRedLights = 0;
		if (inBetween)
		{
			numberOfRedLights = 2;
		}
		else
		{
			numberOfRedLights = 1;
		}
		//Red lights
		int actualNumberOfLights = NumberOfLightsByType(tLight,TSTrafficLight.LightType.Red);
		if (actualNumberOfLights != numberOfRedLights)
		{
			if (actualNumberOfLights > numberOfRedLights)
			{
				RemoveLightsByType(tLight,TSTrafficLight.LightType.Red, actualNumberOfLights-numberOfRedLights);
			}else if (actualNumberOfLights < numberOfRedLights)
			{
				AddLightsByType(tLight,TSTrafficLight.LightType.Red,numberOfRedLights-actualNumberOfLights);
			}
			
		}
		//Other Lights
		CheckOtherTypesOfLights(ref tLight);
	}

	/// <summary>
	/// Checks the other types of lights.
	/// </summary>
	/// <param name="tLight">T light.</param>
	void CheckOtherTypesOfLights(ref TSTrafficLight tLight)
	{
		TSTrafficLight.LightType lightType = TSTrafficLight.LightType.Green;

		for (int i = 0; i < 2;i++)
		{
			switch(i)
			{
			case 1:
				lightType = TSTrafficLight.LightType.Yellow;

				break;
			}
			int actualNumberOfLights = NumberOfLightsByType(tLight,lightType);
			if (manager.greenLightTime != 0 && actualNumberOfLights != 1)
			{
				if (actualNumberOfLights > 1)
				{
					RemoveLightsByType(tLight,lightType, actualNumberOfLights-1);
				}else if (actualNumberOfLights < 1)
				{
					AddLightsByType(tLight,lightType,1);
				}
				
			}
		}
	}

	/// <summary>
	/// Numbers the type of the of lights by.
	/// </summary>
	/// <returns>The of lights by type.</returns>
	int NumberOfLightsByType(TSTrafficLight tLight, TSTrafficLight.LightType lightType)
	{
		int result = 0;
		foreach (TSTrafficLight.TSLight light in tLight.lights)
		{
			if (light.lightType == lightType)
				result++;
		}
		return result;
	}

	/// <summary>
	/// Removes the type of the lights by.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="lightType">Light type.</param>
	/// <param name="amount">Amount.</param>
	void RemoveLightsByType(TSTrafficLight tLight, TSTrafficLight.LightType lightType, int amount)
	{
		int result = 0;
		TSTrafficLight.TSLight light = null;
		for (int i = 0; i < tLight.lights.Count;i++)
		{
			light = tLight.lights[i];
			if (result == amount)break;
			if (light.lightType == lightType)
				tLight.lights.Remove(light);
			result++;
		}
		
	}

	/// <summary>
	/// Adds the type of the lights by.
	/// </summary>
	/// <param name="tLight">T light.</param>
	/// <param name="lightType">Light type.</param>
	/// <param name="amount">Amount.</param>
	void AddLightsByType(TSTrafficLight tLight, TSTrafficLight.LightType lightType, int amount)
	{
		TSTrafficLight.TSLight clone = null;
		for (int i = 0; i < tLight.lights.Count;i++){
			if (tLight.lights[i].lightType == lightType)
			{
				clone = tLight.lights[i];
			}
		}

		for (int i = 0; i < amount;i++)
		{
			AddeNewLight(ref tLight);
			tLight.lights[tLight.lights.Count-1].lightType = lightType;
			if (clone !=null)
			{
				tLight.lights[tLight.lights.Count-1].enableDisableRenderer = clone.enableDisableRenderer;
				tLight.lights[tLight.lights.Count-1].lightGameObject = clone.lightGameObject;
				tLight.lights[tLight.lights.Count-1].lightMeshRenderer = clone.lightMeshRenderer;
				tLight.lights[tLight.lights.Count-1].lightTexture = clone.lightTexture;
				tLight.lights[tLight.lights.Count-1].shaderTexturePropertyName = clone.shaderTexturePropertyName;

			}
		}
		
	}



	void OnSceneGUI() 
	{int i = 0;
		foreach(TSTrafficLight tLight in tlights)
		{
			Handles.Label(tLight.transform.position,tLight.name,EditorStyles.whiteLargeLabel );
			i++;
		}
	}



}
