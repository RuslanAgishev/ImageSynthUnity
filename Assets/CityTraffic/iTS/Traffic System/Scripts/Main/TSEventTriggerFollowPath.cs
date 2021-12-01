using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TSEventTriggerFollowPath : TSEventTrigger {
	public bool disableCarUntilTriggeredByPlayer = false;
	public bool disableCarPlayerSensor = false;
	public bool endEventWithEndingPoint = false;
	public float playerSensorTempDisableTime = 10f;
	
	WaitForSeconds w;

	public override void Awake ()
	{
		base.Awake ();
		w = new WaitForSeconds(playerSensorTempDisableTime);
	}
	
	void OnTriggerEnter()
	{
		if (disableCarUntilTriggeredByPlayer)
		{
			EnableCarAI();
			tAI.reservedForEventTrigger = false;
			if (disableCarPlayerSensor)
				StartCoroutine(TemporaryDisablePlayerSensor());
		}

	}
	public override void InitializeMe ()
	{
		spawnCarOnStartingPoint = true;
	}
	
	IEnumerator EnableCarSensorAtEndPoint()
	{
		while (tAI != Point(eventEndingPoint).carwhoReserved)
		{
			yield return null;
		}
		tAI.playerSensor.enabled = true;
	}
	
	
	IEnumerator TemporaryDisablePlayerSensor()
	{
		yield return null;
		tAI.playerSensor.enabled = false;
		yield return w;
		tAI.playerSensor.enabled = true;
	}
	
	
	public override void SetCar (TSTrafficAI car)
	{
		base.SetCar (car);
		car.ignoreTrafficLight = true;
		if (disableCarUntilTriggeredByPlayer)
		{
			DisableCarAI();
		}
	}
}
