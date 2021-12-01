using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

[CustomEditor(typeof(TSRoadBlock))]
public class TSRoadBlockEditor : Editor {

	TSRoadBlock roadBlock;

	HashSet<int> tempLanes;

//	bool addPoints = false;

//	bool removePoints = false;
	GUISkin myStyle;

	Texture2D tex = new Texture2D(25,20);

	void OnEnable()
	{
		tempLanes = new HashSet<int>();
		roadBlock = (TSRoadBlock)target ;
		if (roadBlock.manager ==null)roadBlock.manager = GameObject.FindObjectOfType<TSMainManager>();
		EditorApplication.update += CheckLanes;
		if (roadBlock.blockingPoints == null || roadBlock.blockingPoints.Length ==0)
			roadBlock.blockingPoints= new TSTrafficLight.TSPointReference[0];
	}

	void OnDisable()
	{
		EditorApplication.update -= CheckLanes;
	}

	public override void OnInspectorGUI()
	{
		base.DrawDefaultInspector();


		if(GUILayout.Button("Clear"))
		{
			roadBlock.blockingPoints = new TSTrafficLight.TSPointReference[0];
		}

		Color defaultC = GUI.color;
		//Legend of the colors
		GUILayout.BeginVertical(GUI.skin.box);
		GUILayout.Label("Color legend");
		GUILayout.BeginHorizontal();
		GUILayout.Label("Lane point UnSelected->", GUILayout.Width(180));

		GUI.color = Color.blue;
		GUILayout.Label(tex,GUILayout.Width(30));
		GUI.color=defaultC;
		GUILayout.Label("Selected->",GUILayout.Width(80));
		GUI.color = Color.magenta;
		GUILayout.Label(tex,GUILayout.Width(30));
		GUILayout.EndHorizontal();
		GUILayout.Space(15);
		GUILayout.BeginHorizontal();
		GUI.color=defaultC;
		GUILayout.Label("Connector point UnSelected->", GUILayout.Width(180));
		GUI.color = Color.grey;
		GUILayout.Label(tex,GUILayout.Width(30));
		GUI.color=defaultC;
		GUILayout.Label("Selected->",GUILayout.Width(80));
		GUI.color = Color.magenta;
		GUILayout.Label(tex,GUILayout.Width(30));
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}


	void OnSceneGUI() 
	{
		if (roadBlock.manager == null) return;
		
		Ray ray = HandleUtility.GUIPointToWorldRay((Event.current.mousePosition));
		RaycastHit hit = new RaycastHit();
		bool rayHit =Physics.Raycast(ray, out hit);
		
		float range = roadBlock.range*roadBlock.range;
		
		if (tempLanes == null) return;
		foreach(int lane in tempLanes){
			CheckPoints(roadBlock.manager.lanes[lane].points, rayHit,hit, range, false, lane,-1);
			if (roadBlock.manager.lanes[lane].connectors.Length > 0)
			{
				for (int p = 0; p < roadBlock.manager.lanes[lane].connectors.Length;p++)
				{
					CheckPoints(roadBlock.manager.lanes[lane].connectors[p].points, rayHit,hit, range, true, lane , p);
				}
			}
		}
		SceneView.RepaintAll();
	}

	int lanesCounter =0;
	int currentLanesCounter=0;
	void CheckLanes()
	{
		int i=0;
		if (lanesCounter>=roadBlock.manager.lanes.Length)lanesCounter=0;
		for(;  lanesCounter < roadBlock.manager.lanes.Length; lanesCounter++){
			i = lanesCounter;
			if (currentLanesCounter > 10)
			{
				currentLanesCounter=0;
				break;
			}
			Bounds	bounds = new Bounds(roadBlock.manager.lanes[i].conectorA,Vector3.one);
			bounds.Encapsulate(roadBlock.manager.lanes[i].conectorB);
			bounds.Encapsulate(roadBlock.manager.lanes[i].points[roadBlock.manager.lanes[i].points.Length/2].point);
			for (int ii =0; ii < roadBlock.manager.lanes[i].connectors.Length;ii++)
			{
				bounds.Encapsulate(roadBlock.manager.lanes[i].connectors[ii].conectorB); 
			}
			Plane[] planes = GeometryUtility.CalculateFrustumPlanes(SceneView.GetAllSceneCameras()[0]);

			if (GeometryUtility.TestPlanesAABB(planes, bounds)){
				tempLanes.Add(i);
			}
			else{
				tempLanes.Remove(i);
			}
			currentLanesCounter++;
		}
	}




	void HighlightPoints(TSPoints[] points)
	{
		Vector3[] listPoints =  new Vector3[points.Length];
		for (int i = 0; i < points.Length;i++)
		{
			listPoints[i] = points[i].point;
		}
		Handles.DrawAAPolyLine(10f, listPoints);
	}


	TSTrafficLight.TSPointReference currentPoint = new TSTrafficLight.TSPointReference();
	void CheckPoints(TSPoints[] points, bool rayHit, RaycastHit hit, float range, bool isConnector, int lane, int connector)
	{
		int controlID = GUIUtility.GetControlID(FocusType.Passive);
		for (int w = 0; w < points.Length;w++) 
		{
			currentPoint.lane = lane;
			currentPoint.connector = connector;
			currentPoint.point = w;
			bool selected = false;
			if ((points[w].point - roadBlock.transform.position).sqrMagnitude < range){
				if (rayHit && (new Vector3(points[w].point.x,hit.point.y,points[w].point.z) - hit.point).magnitude <= 0.5f)
				{
					if ((Event.current.type ==  EventType.MouseDown) && Event.current.button ==0  ){
						TSTrafficLight.TSPointReference point = new TSTrafficLight.TSPointReference();
						point.connector = connector;
						point.lane = lane;
						point.point = w;
							if (!Contains(point,roadBlock.blockingPoints)){
								roadBlock.blockingPoints = roadBlock.blockingPoints.Add(point);
							}else{
								int index = GetListIndex(point, roadBlock.blockingPoints);
								if (index != -1){
								roadBlock.blockingPoints=roadBlock.blockingPoints.Remove(roadBlock.blockingPoints[index]);
							}
						}
						selected = true;
						GUIUtility.hotControl = controlID;						
					}else
					{
						Handles.color = Color.red;						
					}
					Handles.color = Color.yellow;
					Handles.DrawSolidDisc(points[w].point,Vector3.up, 0.5f);
				}else
				{
					if (Contains(currentPoint, roadBlock.blockingPoints)){
						Handles.color = Color.magenta;
					}else
					{
						if (isConnector)
							Handles.color = Color.grey;
						else
							Handles.color = Color.blue;
					}
					Handles.DrawSolidDisc(points[w].point,Vector3.up, 0.5f);
				}
				
			}
			if (selected){ 
				GUIUtility.hotControl = 0;
			}
		}
		
	}
	
	
	bool Contains(TSTrafficLight.TSPointReference point, TSTrafficLight.TSPointReference[] list)
	{
		for (int i =0; i < list.Length; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return true;
		}
		return false;
	}
	
	int GetListIndex(TSTrafficLight.TSPointReference point, TSTrafficLight.TSPointReference[] list)
	{
		for (int i =0; i < list.Length; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return i;
		}
		return -1;
	}
}
