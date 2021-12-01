using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TSEventTriggerSuddenBrake : TSEventTrigger {
	public float stopTime = 10f;
	WaitForSeconds w1;

	void OnTriggerEnter()
	{
		if (!isTriggered)
			StartCoroutine(CheckForPointsCarReference());
	}

	public override void Awake ()
	{
		base.Awake ();
		w1 = new WaitForSeconds(stopTime);
	}
	
	public override void InitializeMe ()
	{
		spawnCarOnStartingPoint = false;
	}

	IEnumerator CheckForPointsCarReference()
	{
		isTriggered =true;
		while (tAI == null)
		{
			if ( manager.lanes[startingPoint.lane].points[startingPoint.point].carwhoReserved !=null)
			{
				tAI = manager.lanes[startingPoint.lane].points[startingPoint.point].carwhoReserved;
				nav = tAI.GetComponent<TSNavigation>();
				break;
			}
			yield return null;
		}
		DisableCarAI();
		tAI.GetComponent<TSSimpleCar>().OnAIUpdate(0,1,0,false);
		yield return w1;
		EnableCarAI();
		tAI = null;
	}
}
