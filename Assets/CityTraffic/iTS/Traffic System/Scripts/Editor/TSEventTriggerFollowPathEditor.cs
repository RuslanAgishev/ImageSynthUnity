using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;

[CustomEditor(typeof(TSEventTriggerFollowPath))]
public class TSEventTriggerFollowPathEditor : Editor {

	TSEventTrigger trigger;
	TSMainManager manager;
	HashSet<int> tempLanes = new HashSet<int>();
	Collider collider;

	void OnEnable()
	{
		trigger = (TSEventTrigger)target;
		trigger.spawnCarOnStartingPoint = true;
		EditorUtility.SetDirty(trigger);
		manager = GameObject.FindObjectOfType<TSMainManager>();
		collider = trigger.GetComponentInChildren<Collider>();
		EditorApplication.update += CheckLanes;
	}


	int currentSelection =0;

	public override void OnInspectorGUI ()
	{
		collider = trigger.GetComponentInChildren<Collider>();
		EditorGUI.BeginChangeCheck();
		base.OnInspectorGUI ();

		currentSelection = GUILayout.Toolbar(currentSelection,new string[]{"Spawning Point","Ending Point","AI Path"});
		if (EditorGUI.EndChangeCheck())
			EditorUtility.SetDirty(trigger);
		if (collider == null)
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

			if (currentSelection==0)
			{
				CheckPoints(ref trigger.startingPoint, manager.lanes[lane].points, rayHit,hit, range, false, lane,-1);
			}
			else if (currentSelection ==1)
			{
				CheckPoints(ref trigger.eventEndingPoint, manager.lanes[lane].points, rayHit,hit, range, false, lane,-1);
			}
			else
			{
				CheckPointsPath(manager.lanes[lane].points, rayHit,hit, range, false, lane,-1);
			}


			if (manager.lanes[lane].connectors.Length > 0)
			{
				for (int p = 0; p < manager.lanes[lane].connectors.Length;p++)
				{
					if (currentSelection==0)
					{
						CheckPoints(ref trigger.startingPoint, manager.lanes[lane].connectors[p].points, rayHit,hit, range, true, lane , p);
					}else if (currentSelection ==1)
					{
						CheckPoints(ref trigger.eventEndingPoint, manager.lanes[lane].connectors[p].points, rayHit,hit, range, false, lane , p);
					}else
					{
						CheckPointsPath(manager.lanes[lane].connectors[p].points, rayHit,hit, range, true, lane , p);
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
	
	TSNavigation.TSNextLaneSelection currentPath = new TSNavigation.TSNextLaneSelection();
	void CheckPointsPath(TSPoints[] points, bool rayHit, RaycastHit hit, float range, bool isConnector, int lane, int connector)
	{
		int controlID = GUIUtility.GetControlID(FocusType.Passive);
//		bool isInRange = false;
		for (int w = 0; w < points.Length;w++) 
		{
			currentPath.nextLane = lane;
			currentPath.nextConnector = connector;
			currentPath.isConnector = isConnector;
			bool selected = false;
			{
				if ( rayHit && (points[w].point - hit.point).magnitude <= manager.resolutionConnectors/2f)
				{
					if ((Event.current.type ==  EventType.MouseDown) && Event.current.button ==0  ){
						TSNavigation.TSNextLaneSelection newPath = new TSNavigation.TSNextLaneSelection();
						newPath.nextConnector = connector;
						newPath.nextLane = lane;
						newPath.isConnector = isConnector;
						
						
						if (!trigger.carPredefinedPath.Contains(newPath)){
							trigger.carPredefinedPath.Add(newPath);
						}
						else{

							if (trigger.carPredefinedPath.Contains(newPath)){
								trigger.carPredefinedPath.Remove(newPath);
							}
						}
						EditorUtility.SetDirty(trigger);
						selected = true;
						GUIUtility.hotControl = controlID;
						
					}else
					{
						Handles.color = Color.red;
						
					}
//					Handles.color = Color.blue;

				}else
				{
					if (w ==0){
						if (connector == -1)
						{
							if (trigger.carPredefinedPath.Contains(currentPath)){
								Handles.color = Color.yellow;
							}else
							{
								Handles.color = Color.green;
							}

							Handles.Label(points[points.Length/2].point,"Lane " + lane,EditorStyles.whiteLargeLabel);
							
						}else
						{
							if (trigger.carPredefinedPath.Contains(currentPath)){
								Handles.color = Color.magenta;
							}else
							{
								Handles.color = Color.grey;
							}
							Handles.Label(points[points.Length/2].point,"Connector " + connector,EditorStyles.whiteLargeLabel);
						}
						
					}
				}
				
			}

			if (selected){ GUIUtility.hotControl = 0;
			}
		}
//		if (isInRange){
			HighlightPoints(points);
			if (points.Length > 4 && !isConnector){
				Vector3[] p = new Vector3[4];
				int index = points.Length/2;
				Quaternion tempDir = Quaternion.LookRotation((points[index+1 < manager.lanes[lane].points.Length? index+1:index].point-points[index].point));
				p[0] = points[index].point + tempDir * Vector3.right * manager.lanes[lane].laneWidth ;
				p[1]= points[index].point + tempDir * Vector3.forward * 5*manager.scaleFactor ;
				p[2] = points[index].point + tempDir * -Vector3.right  * manager.lanes[lane].laneWidth ;
				p[3] = points[index].point ;
				Handles.DrawSolidRectangleWithOutline(p, Handles.color,Handles.color);
			}
//		}
		
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
