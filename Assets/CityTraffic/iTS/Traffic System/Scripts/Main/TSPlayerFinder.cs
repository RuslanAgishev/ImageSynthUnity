using UnityEngine;
using System.Collections;

public class TSPlayerFinder : MonoBehaviour {

	TSMainManager manager;

	public TSPoints currentPoint;

	public int laneFound;
	public int connectorFound;
	public int pointFound;

	TSPoints newPoint;

	Transform myTransform;
	// Use this for initialization
	void Start () {
		manager = GameObject.FindObjectOfType<TSMainManager>();
		myTransform = transform;
		GetNearestPointBruteSearch();
	}
	
	// Update is called once per frame
	void Update () {
		FindNearestPoint();
	}


	void FindNearestPoint()
	{
		float currentPDistance = (currentPoint.point - myTransform.position).sqrMagnitude;
		newPoint = currentPoint;
		for (int i =0;i < currentPoint.nearbyPoints.Length;i++)
		{
			int lane = currentPoint.nearbyPoints[i].lane;
			int connector = currentPoint.nearbyPoints[i].connector;
			int point = currentPoint.nearbyPoints[i].pointIndex;
			currentPDistance =  GetNewDistance(lane,connector,point, currentPDistance);
		}
		currentPoint = newPoint;
	}

	float GetNewDistance(int lane, int connector, int pointindex, float currentPDistance)
	{
		TSPoints point = null;
		if (connector ==-1)
		{
			point = manager.lanes[lane].points[pointindex];
		}else
		{
			point = manager.lanes[lane].connectors[connector].points[pointindex];
		}

		float newD =(point.point - myTransform.position).sqrMagnitude;
		if (newD < currentPDistance)
		{
			newPoint = point;
			laneFound = lane;
			connectorFound = connector;
			pointFound = pointindex;
			return newD;
		}
		return currentPDistance;
	}



	void GetNearestPointBruteSearch()
	{
		float mindDist=float.MaxValue;
		for (int i = 0; i < manager.lanes.Length;i++)
		{
			for (int y = 0;y< manager.lanes[i].points.Length;y++)
			{
				float newD =(manager.lanes[i].points[y].point - myTransform.position).sqrMagnitude;
				if (newD < mindDist)
				{
					laneFound = i;
					connectorFound = -1;
					pointFound = y;
					mindDist = newD;
					currentPoint = manager.lanes[i].points[y];
				}
			}

			for (int r =0; r < manager.lanes[i].connectors.Length;r++)
			{
				for (int p=0;p<manager.lanes[i].connectors[r].points.Length;p++)
				{
					float newD =(manager.lanes[i].connectors[r].points[p].point - myTransform.position).sqrMagnitude;
					if (newD < mindDist)
					{
						laneFound = i;
						connectorFound = r;
						pointFound = p;
						mindDist = newD;
						currentPoint = manager.lanes[i].connectors[r].points[p];
					}
				}
			}

		}
	}
}
