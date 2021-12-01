using UnityEngine;
using System.Collections;
using UnityEditor;
using System.IO;



	public class TSSimpleCarRiggerMenu : Editor {

	[MenuItem("GameObject/iTS/Simple Vehicle/Add Empty")]
	static void AddCarPhysics () {
		TSSimpleCarRigger myWindow = (TSSimpleCarRigger)EditorWindow.CreateInstance<TSSimpleCarRigger>();
		myWindow.title = "Car Rig - Wizard";
		myWindow.Init();
		myWindow.ShowUtility();
	}

	[MenuItem("GameObject/iTS/iTS Manager/Add iTS Manager to the scene")]
	static void AddMainManager () {
	
		if (FindObjectOfType(typeof(TSMainManager))!= null)
		{
			EditorUtility.DisplayDialog("Warning!","This scene already contains an iTS Manager!","Ok");
		}else
		{
			GameObject newiTSManager = new GameObject();
			newiTSManager.name = "iTSManager";
			newiTSManager.AddComponent<TSMainManager>();
		}
	}

	[MenuItem("GameObject/iTS/iTS Manager/Add iTS Traffic Spawner to the scene")]
	static void AddSpawner () {
		
		if (FindObjectOfType(typeof(TSTrafficSpawner))!= null)
		{
			EditorUtility.DisplayDialog("Warning!","This scene already contains an iTS Traffic Spawner!","Ok");
		}else
		{
			GameObject newiTSManager = new GameObject();
			newiTSManager.name = "TrafficSpawner";
			newiTSManager.AddComponent<TSTrafficSpawner>();
		}
	}


	[MenuItem("GameObject/iTS/iTS Manager/Add iTS Traffic Volume to the scene")]
	static void AddTrafficVolume () {
		GameObject newiTSManager = new GameObject();
		newiTSManager.name = "TrafficVolume";
		if (Selection.activeGameObject != null)
			newiTSManager.transform.parent = Selection.activeGameObject.transform;
		newiTSManager.AddComponent<TSTrafficVolume>();
		newiTSManager.GetComponent<BoxCollider>().isTrigger = true;
	}



}
