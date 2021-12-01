using UnityEngine;
using System.Collections;
using UnityEditor;

[CustomEditor(typeof(TSTrafficLightCheck))]
public class TSTrafficLightCheckEditor : Editor {

	void OnEnable()
	{
		TSMainManagerEditor[] mainmanagerEditors = Resources.FindObjectsOfTypeAll<TSMainManagerEditor>();
		foreach(TSMainManagerEditor editor in mainmanagerEditors)
		{
			editor.onLaneDeleted += CheckTraficLights;
		}
	}

	void OnDisable()
	{
		TSMainManagerEditor[] mainmanagerEditors = Resources.FindObjectsOfTypeAll<TSMainManagerEditor>();
		foreach(TSMainManagerEditor editor in mainmanagerEditors)
		{
			editor.onLaneDeleted -= CheckTraficLights;
		}

	}

	void CheckTraficLights(int deletedLane)
	{
		TSTrafficLight[] allTrafficLights = GameObject.FindObjectsOfType<TSTrafficLight>();
		if (allTrafficLights == null || allTrafficLights.Length ==0) return;
		for (int i= 0 ; i < allTrafficLights.Length; i++)
		{
			for (int y = 0; y < allTrafficLights[i].pointsNormalLight.Count;y++)
			{
				if (allTrafficLights[i].pointsNormalLight[y].lane > deletedLane)
				{
					allTrafficLights[i].pointsNormalLight[y].lane--;
				}
			}
		}
	}
}
