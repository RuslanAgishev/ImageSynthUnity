using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using System.Linq;
using System.Xml.Serialization;
using System.IO;

[CustomEditor(typeof(TSTrafficLight))]
public class TSTrafficLightEditor : Editor {

	TSTrafficLight tLight;

	private GUIContent[] dispOp;
	private string[] toolbar1Contents = {"Lights","Connectors","Settings"};
	static int menuSelection = 0;
	static bool addPoints;
	static bool removePoints;
	static bool addPointsT;
	static bool removePointsT;
	Color defaultColor = Color.black;
	TSTrafficLight.TSPointReference currentPoint = new TSTrafficLight.TSPointReference();

	public void OnEnable() 
	{ 
		defaultColor =GUI.color;
		tLight = (TSTrafficLight)target;
		if (tLight.manager == null)
			tLight.manager = GameObject.FindObjectOfType(typeof(TSMainManager)) as TSMainManager;
	}




	public override void OnInspectorGUI()
	{

		if (GUILayout.Button("Stop all coroutines"))
		{
			tLight.StopAllCoroutines();
		}

		GUILayout.Space(15);

		menuSelection = GUILayout.Toolbar(menuSelection, toolbar1Contents);

		switch(menuSelection)
		{
		case 0:
			LightsOptions();
			break;
		case 1:
			PointsOptions();
			break;
		case 2:
			Settings();
			break;

		}

		GUILayout.Space(20);
		GUILayout.BeginVertical("Current Selected Connectors",EditorStyles.textField);
		GUILayout.Space(20);
		EditorGUILayout.LabelField((tLight.pointsNormalLight.Count).ToString()); 
		GUILayout.EndVertical();
		if (GUI.changed ){
			EditorUtility.SetDirty(tLight);
		} 
	}

	void LightsOptions()
	{
		GUILayout.Space(15);
		
		if (GUILayout.Button("Add new light"))
		{
			tLight.lights.Add(new TSTrafficLight.TSLight());
			tLight.lights[tLight.lights.Count-1].lightTime = 15;
			tLight.lights[tLight.lights.Count-1].shaderTexturePropertyName = "_MainTex";
		}
		GUILayout.Space(15);
		
		for (int i = 0; i < tLight.lights.Count; i++)
		{
			if (Application.isPlaying && tLight.lightToPlay == i)
			{
				switch(tLight.lights[tLight.lightToPlay].lightType)
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
			GUILayout.BeginVertical("Light "+tLight.lights[i].lightType + " " + i,EditorStyles.textField);
			GUILayout.Space(20);

			if (GUILayout.Button("Up", EditorStyles.miniButton, GUILayout.Width(30)) && i !=0){
				Move<TSTrafficLight.TSLight>(tLight.lights,i,i-1);

			}
			if (GUILayout.Button("Dw", EditorStyles.miniButton, GUILayout.Width(30))){
				Move<TSTrafficLight.TSLight>(tLight.lights,i,i+1);
			}


			if (GUILayout.Button("Delete", EditorStyles.miniButton, GUILayout.Width(80))){
				tLight.lights.Remove(tLight.lights[i]);
				break;
			}



			tLight.lights[i].lightType = (TSTrafficLight.LightType) EditorGUILayout.EnumPopup("Light type",tLight.lights[i].lightType);
			tLight.lights[i].lightMeshRenderer = (MeshRenderer)EditorGUILayout.ObjectField("Mesh Renderer",tLight.lights[i].lightMeshRenderer, typeof(MeshRenderer),true);
			tLight.lights[i].enableDisableRenderer = EditorGUILayout.Toggle("Enable / Disable Renderer?",tLight.lights[i].enableDisableRenderer);
			tLight.lights[i].lightGameObject = (GameObject)EditorGUILayout.ObjectField("Optional GameObject",tLight.lights[i].lightGameObject, typeof(GameObject),true);
			tLight.lights[i].lightTexture = (Texture2D)EditorGUILayout.ObjectField("Texture",tLight.lights[i].lightTexture, typeof(Texture2D),false );
			tLight.lights[i].shaderTexturePropertyName = EditorGUILayout.TextField("Shader Texture name",tLight.lights[i].shaderTexturePropertyName);
			tLight.lights[i].lightTime = EditorGUILayout.FloatField("Time",tLight.lights[i].lightTime);
			GUILayout.Space(5);
			GUILayout.EndVertical();
			GUILayout.Space(15);
		}
		

	}


	void Settings()
	{
		GUILayout.Space(15);
		tLight.yellowLightsStopTraffic = EditorGUILayout.Toggle("Yellow Lights Stop Traffic?",tLight.yellowLightsStopTraffic);
		tLight.lightRange = EditorGUILayout.Slider("Light Range",tLight.lightRange,1f,250f);
		GUILayout.Space(15);
		SaveLoadSchedule();
		SaveLoadConnectorMapping();
	}

	void SaveLoadSchedule()
	{
		GUILayout.BeginVertical(GUI.skin.box);
		GUILayout.Label("Save/load traffic schedule");
		GUILayout.BeginHorizontal();
		
		if (GUILayout.Button("Save", GUILayout.Width(60)))
		{
			string path = EditorUtility.SaveFilePanel("Save traffic light settings as",Application.dataPath,tLight.name + "schedule","xml");
			if (path.Length !=0)
				Save(path);
		}
		if (GUILayout.Button("Load", GUILayout.Width(60)))
		{
			string path = EditorUtility.OpenFilePanel("Load traffic light settings",Application.dataPath,"xml");
			if (path.Length!=0){
				tLight.lights = Load(path);
			}
		}
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	void SaveLoadConnectorMapping()
	{
		GUILayout.BeginVertical(GUI.skin.box);
		GUILayout.Label("Save/load traffic connector Mapping");
		GUILayout.BeginHorizontal();
		
		if (GUILayout.Button("Save", GUILayout.Width(60)))
		{
			string path = EditorUtility.SaveFilePanel("Save traffic light mapping as",Application.dataPath,tLight.name + "mapping","xml");
			if (path.Length !=0)
				SaveMapping(path);
		}
		if (GUILayout.Button("Load", GUILayout.Width(60)))
		{
			string path = EditorUtility.OpenFilePanel("Load traffic light mapping",Application.dataPath,"xml");
			if (path.Length!=0){
				tLight.pointsNormalLight = LoadMapping(path);
			}
		}
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	void PointsOptions()
	{
		GUILayout.BeginVertical("Normal Traffic Lights Connectors",EditorStyles.textField);
		GUILayout.Space(20);
		addPoints = GUILayout.Toggle(addPoints, "Add Connectors", EditorStyles.miniButton);
		if (addPoints){
			removePoints = false;
			addPointsT = false;
			removePointsT = false;
		}
		removePoints = GUILayout.Toggle(removePoints,"Remove Connectors",EditorStyles.miniButton);
		if (removePoints){
			addPoints = false;
			addPointsT = false;
			removePointsT = false;
		}
		if (GUILayout.Button("Clear"))
		{
			int selection =  EditorUtility.DisplayDialogComplex("Data would get Cleared!","Do you want to clear all the selected connectors?","Yes","No","Cancel");
			if (selection == 0)
			tLight.pointsNormalLight.Clear();
		}
		GUILayout.Space(15);

		for (int i = 0; i < tLight.pointsNormalLight.Count;i++)
		{
			GUILayout.BeginVertical(EditorStyles.textField);
			if (GUILayout.Button("Remove",EditorStyles.miniButton, GUILayout.Width(65)))
			{
				tLight.pointsNormalLight.Remove(tLight.pointsNormalLight[i]);
				break;
			}
			GUILayout.BeginHorizontal();
			GUILayout.Label("Lane: ");
			GUILayout.Label(tLight.pointsNormalLight[i].lane.ToString());
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Connector: ");
			GUILayout.Label(tLight.pointsNormalLight[i].connector.ToString());
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			
			
		}

		GUILayout.EndVertical();



		GUILayout.Space(20);
		if (GUI.changed)EditorUtility.SetDirty(tLight);
	}






	void OnSceneGUI() 
	{
		if (tLight.manager == null) return;

		Ray ray = HandleUtility.GUIPointToWorldRay((Event.current.mousePosition));
		RaycastHit hit = new RaycastHit();
		bool rayHit =Physics.Raycast(ray, out hit);

		float range = tLight.lightRange*tLight.lightRange;
		
		if (tLight.manager.lanes == null) return;
		for(int i = 0; i < tLight.manager.lanes.Length; i++){
			Handles.color = Color.green;
			CheckPoints(tLight.manager.lanes[i].points, rayHit,hit, range, false, i,-1);
			if (tLight.manager.lanes[i].connectors.Length > 0)
			{
				for (int p = 0; p < tLight.manager.lanes[i].connectors.Length;p++)
				{
					CheckPoints(tLight.manager.lanes[i].connectors[p].points, rayHit,hit, range, true, i , p);

				}
			}
		}
		SceneView.RepaintAll();
	}
		//OnSceneGUI end

	 
	void HighlightPoints(TSPoints[] points)
	{
		Vector3[] listPoints =  new Vector3[points.Length];
		for (int i = 0; i < points.Length;i++)
		{
			listPoints[i] = points[i].point;
		}
		Handles.DrawAAPolyLine(tLight.manager.visualLinesWidth, listPoints);
	}

	void CheckPoints(TSPoints[] points, bool rayHit, RaycastHit hit, float range, bool isConnector, int lane, int connector)
	{
		int controlID = GUIUtility.GetControlID(FocusType.Passive);


		for (int w = 0; w < points.Length;w++) 
		{
			currentPoint.lane = lane;
			currentPoint.connector = connector;
			currentPoint.point = w;
			bool selected = false;
			if ((points[w].point - tLight.transform.position).sqrMagnitude < range){
				if (connector != -1 && rayHit && (points[w].point - hit.point).magnitude <= tLight.manager.resolutionConnectors/2f)
				{
					if ((Event.current.type ==  EventType.MouseDown) && Event.current.button ==0  && menuSelection == 1){
						TSTrafficLight.TSPointReference point = new TSTrafficLight.TSPointReference();
						point.connector = connector;
						point.lane = lane;
						point.point = 0;


						if (addPoints && !addPointsT)	{
							if (!Contains(point,tLight.pointsNormalLight)){
								tLight.pointsNormalLight.Add(point);
							}
							EditorUtility.SetDirty(tLight);
						}

						if (removePoints && !removePointsT)	{
							int index = GetListIndex(point, tLight.pointsNormalLight);
							if (index != -1){
								tLight.pointsNormalLight.Remove(tLight.pointsNormalLight[index]);
							}
							EditorUtility.SetDirty(tLight);
						}
						selected = true;
						GUIUtility.hotControl = controlID;

					}else
					{
						Handles.color = Color.red;

					}
					Handles.color = Color.blue;
					HighlightPoints(points);
				}else
				{
					if (w ==0){
						if (connector == -1)
						{
							Handles.color = Color.green;
							Handles.Label(points[points.Length/2].point,"Lane " + lane,EditorStyles.whiteLargeLabel);

						}else
						{
							if (Contains(currentPoint, tLight.pointsNormalLight)){
								Handles.color = Color.magenta;
							}else
							{
								if (isConnector)
									Handles.color = Color.grey;
								else
									Handles.color = Color.blue;
							}
							Handles.Label(points[points.Length/2].point,"Connector " + connector,EditorStyles.whiteLargeLabel);
						}
						HighlightPoints(points);

					}
				}

			}
			if (selected){ GUIUtility.hotControl = 0;
			}
		}

	}


	bool Contains(TSTrafficLight.TSPointReference point, List<TSTrafficLight.TSPointReference> list)
	{
		for (int i =0; i < list.Count; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return true;
		}
		return false;
	}

	int GetListIndex(TSTrafficLight.TSPointReference point, List<TSTrafficLight.TSPointReference> list)
	{
		for (int i =0; i < list.Count; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return i;
		}
		return -1;
	}

	void Move<T>(List<T> list, int oldIndex, int newIndex)
	{
		T aux = list[newIndex];
		list[newIndex] = list[oldIndex];
		list[oldIndex] = aux;
	}




	public void Save(string path)
	{
		var serializer = new XmlSerializer(typeof(List<TSTrafficLight.TSLight>));
		using(StreamWriter stream = new StreamWriter( new FileStream(path, FileMode.Create), System.Text.Encoding.UTF8 ))
		{
			serializer.Serialize(stream, tLight.lights);
		}
	}
	
	public List<TSTrafficLight.TSLight> Load(string path)
	{
		var serializer = new XmlSerializer(typeof(List<TSTrafficLight.TSLight>));
		using(StreamReader stream = new StreamReader( new FileStream(path, FileMode.Open),System.Text.Encoding.UTF8))
		{
			return (serializer.Deserialize(stream) as List<TSTrafficLight.TSLight>);
		}
	}


	public void SaveMapping(string path)
	{
		var serializer = new XmlSerializer(typeof(List<TSTrafficLight.TSPointReference>));
		using(StreamWriter stream = new StreamWriter( new FileStream(path, FileMode.Create), System.Text.Encoding.UTF8 ))
		{
			serializer.Serialize(stream, tLight.pointsNormalLight);
		}
	}
	
	public List<TSTrafficLight.TSPointReference> LoadMapping(string path)
	{
		var serializer = new XmlSerializer(typeof(List<TSTrafficLight.TSPointReference>));
		using(StreamReader stream = new StreamReader( new FileStream(path, FileMode.Open),System.Text.Encoding.UTF8))
		{
			return (serializer.Deserialize(stream) as List<TSTrafficLight.TSPointReference>);
		}
	}

}
