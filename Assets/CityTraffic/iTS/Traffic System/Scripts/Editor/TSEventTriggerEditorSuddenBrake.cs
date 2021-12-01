using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;

[CustomEditor(typeof(TSEventTriggerSuddenBrake))]
public class TSEventTriggerEditorSuddenBrake : Editor {

	TSEventTrigger trigger;
	TSMainManager manager;
	HashSet<int> tempLanes = new HashSet<int>();
	Collider collider;

	void OnEnable()
	{
		trigger = (TSEventTrigger)target;
		manager = GameObject.FindObjectOfType<TSMainManager>();
		EditorApplication.update += CheckLanes;
		collider = trigger.GetComponentInChildren<Collider>();
	}


	int currentSelection =0;

	public override void OnInspectorGUI ()
	{
		collider = trigger.GetComponentInChildren<Collider>();
		EditorGUI.BeginChangeCheck();
		base.OnInspectorGUI ();
		if (EditorGUI.EndChangeCheck())
			EditorUtility.SetDirty(trigger);
		if (collider == null )
		{
			EditorGUILayout.HelpBox("Please add a collider (trigger) to this Game Object!", MessageType.Warning);
		}
		if (collider != null && !collider.isTrigger)
		{
			EditorGUILayout.HelpBox("Please make your colliders attached to this game object be triggers!", MessageType.Warning);
		}
	}


	
	void OnSceneGUI() 
	{
		if (manager == null) return;
		
		Ray ray = HandleUtility.GUIPointToWorldRay((Event.current.mousePosition));
		RaycastHit hit = new RaycastHit();
		bool rayHit =Physics.Raycast(ray, out hit);
		
		float range = trigger.range*trigger.range;
		
		if (tempLanes == null) return;
		foreach(int lane in tempLanes){

				CheckPoints(ref trigger.startingPoint, manager.lanes[lane].points, rayHit,hit, range, false, lane,-1);
			if (manager.lanes[lane].connectors.Length > 0)
			{
				for (int p = 0; p < manager.lanes[lane].connectors.Length;p++)
				{
					if (currentSelection==0)
					{
						CheckPoints(ref trigger.startingPoint, manager.lanes[lane].connectors[p].points, rayHit,hit, range, true, lane , p);
					}
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
		if (lanesCounter>=manager.lanes.Length)lanesCounter=0;
		for(;  lanesCounter < manager.lanes.Length; lanesCounter++){
			i = lanesCounter;
			if (currentLanesCounter > 50)
			{
				currentLanesCounter=0;
				break;
			}
			Bounds	bounds = new Bounds(manager.lanes[i].conectorA,Vector3.one);
			bounds.Encapsulate(manager.lanes[i].conectorB);
			bounds.Encapsulate(manager.lanes[i].points[manager.lanes[i].points.Length/2].point);
			for (int ii =0; ii < manager.lanes[i].connectors.Length;ii++)
			{
				bounds.Encapsulate(manager.lanes[i].connectors[ii].conectorB); 
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
	
	
	void CheckPoints(ref TSEventTrigger.TSPointReference currentPoint, TSPoints[] points, bool rayHit, RaycastHit hit, float range, bool isConnector, int lane, int connector)
	{
		int controlID = GUIUtility.GetControlID(FocusType.Passive);
		for (int w = 0; w < points.Length;w++) 
		{
			bool selected = false;
			if ((points[w].point - trigger.transform.position).sqrMagnitude < range){
				TSEventTrigger.TSPointReference point = new TSEventTrigger.TSPointReference();
				point.connector = connector;
				point.lane = lane;
				point.point = w;
				if (rayHit && (new Vector3(points[w].point.x,hit.point.y,points[w].point.z) - hit.point).magnitude <= 0.5f)
				{
					if ((Event.current.type ==  EventType.MouseDown) && Event.current.button ==0 && !isConnector ){

						if (currentPoint.lane == point.lane && currentPoint.connector == point.connector && currentPoint.point ==  point.point){
							currentPoint = null;
						}else{
							currentPoint = point;
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
					if (currentPoint != null && currentPoint.lane == point.lane && currentPoint.connector == point.connector && currentPoint.point ==  point.point){
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
	


	bool Contains(TSEventTrigger.TSPointReference point, TSEventTrigger.TSPointReference[] list)
	{
		for (int i =0; i < list.Length; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return true;
		}
		return false;
	}
	
	int GetListIndex(TSEventTrigger.TSPointReference point, TSEventTrigger.TSPointReference[] list)
	{
		for (int i =0; i < list.Length; i++)
		{
			if (point.lane == list[i].lane && point.connector == list[i].connector && point.point == list[i].point)
				return i;
		}
		return -1;
	}

}
