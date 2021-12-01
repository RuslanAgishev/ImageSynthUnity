using UnityEngine;
using System.Collections;

public class TSRoadBlock : MonoBehaviour {


	public TSTrafficLight.TSPointReference[] blockingPoints = new TSTrafficLight.TSPointReference[0];

	public TSMainManager manager;

	public float roadBlockAheadDistance = 40f;

	public float range = 10;
	private int myID;

	// Use this for initialization
	void Awake () {
		myID=GetInstanceID();
		if (manager ==null)
			manager = GameObject.FindObjectOfType<TSMainManager>();
	}

	void OnEnable()
	{
		if (manager !=null)
			BlockPoints();
	}

	void OnDisable()
	{
		if (manager !=null){
			UnBlockPoints();
		}
	}

	public void BlockPoints()
	{
		for (int i =0; i < blockingPoints.Length;i++)
		{
			SetPointReservationID(blockingPoints[i],myID);
		}
	}



	public void UnBlockPoints()
	{
		for (int i =0; i < blockingPoints.Length;i++)
		{
			SetPointReservationID(blockingPoints[i],0);
		}
	}


	void SetPointReservationID(TSTrafficLight.TSPointReference point, int reservationID)
	{
		TSTrafficLight.TSPointReference roadBlockPoint = new TSTrafficLight.TSPointReference();
		if (point.connector == -1)
		{
			manager.lanes[point.lane].points[point.point].reservationID = reservationID;
			manager.lanes[point.lane].points[point.point].carwhoReserved =null;
			roadBlockPoint = point;
		}
		else
		{
			manager.lanes[point.lane].connectors[point.connector].points[point.point].reservationID = reservationID;
			manager.lanes[point.lane].connectors[point.connector].points[point.point].carwhoReserved =null;
			manager.lanes[point.lane].points[manager.lanes[point.lane].points.Length-1].carwhoReserved =null;
			manager.lanes[point.lane].points[manager.lanes[point.lane].points.Length-1].reservationID =reservationID;
			roadBlockPoint.connector = -1;
			roadBlockPoint.lane = point.lane;
			roadBlockPoint.point = manager.lanes[point.lane].points.Length-1;
		}
		SetRoadBlockAhead(point, (reservationID !=0));
	}

	void SetRoadBlockAhead(TSTrafficLight.TSPointReference point,bool setRoadBlock)
	{
		float dist =0;
		int currentPoint = point.point;
		while (dist < roadBlockAheadDistance && currentPoint >=0)
		{
			manager.lanes[point.lane].points[currentPoint].roadBlockAhead = setRoadBlock;
			dist +=manager.lanes[point.lane].points[currentPoint].distanceToNextPoint;
			currentPoint--;
		}
	}


}
