using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Xml.Serialization;

/// <summary>
/// TS traffic light class.  This class is responsible for managing the lights of a Traffic light and
/// making the cars stop on the selected junctions
/// </summary>
public class TSTrafficLight : MonoBehaviour {


	[System.Serializable]
	public class TSLight{
		/// <summary>
		/// The light timer.
		/// </summary>
		[XmlElement("lightTimer")]
		public float lightTimer = 0f;

		/// <summary>
		/// The type of the light.
		/// </summary>
		[XmlElement("lightType")]
		public TSTrafficLight.LightType lightType;

		/// <summary>
		/// The light time.
		/// </summary>
		[XmlElement("lightTime")]
		public float lightTime = 20f;

		/// <summary>
		/// The light texture.
		/// </summary>
		[XmlIgnore]
		public Texture2D lightTexture;

		/// <summary>
		/// The light mesh renderer.
		/// </summary>
		[XmlIgnore]
		public MeshRenderer lightMeshRenderer;

		/// <summary>
		/// The enable disable renderer.
		/// </summary>
		[XmlElement("enableDisableRenderer")]
		public bool enableDisableRenderer = false;

		/// <summary>
		/// The light game object.
		/// </summary>
		[XmlIgnore]
		public GameObject lightGameObject;

		/// <summary>
		/// The name of the shader texture property.
		/// </summary>
		[XmlElement("shaderTexturePropertyName")]
		public string shaderTexturePropertyName = "_MainTex";
	}

	[System.Serializable]
	public class TSPointReference{
		/// <summary>
		/// The lane.
		/// </summary>
		[XmlElement("lane")]
		public int lane;

		/// <summary>
		/// The connector.
		/// </summary>
		[XmlElement("connector")]
		public int connector;

		/// <summary>
		/// The point.
		/// </summary>
		[XmlElement("point")]
		public int point;
	}

	//Marmoset is _Illum for texture property name
	[System.Serializable]
	public enum LightType{
		Green,
		Red,
		Yellow,
		NoLights,
	}

	/// <summary>
	/// The lights.
	/// </summary>
	[SerializeField]
	[XmlArray("Lights")]
	[XmlArrayItem("Light")]
	public List<TSLight> lights = new List<TSLight>();

	/// <summary>
	/// The manager.
	/// </summary>
	[SerializeField]
	public TSMainManager manager; 

	/// <summary>
	/// The last lane.
	/// </summary>
	private int lastLane;

	/// <summary>
	/// The last point.
	/// </summary>
	private int lastPoint;

	/// <summary>
	/// The last connector.
	/// </summary>
	private int lastConnector;

	/// <summary>
	/// The traffic light ID.
	/// </summary>
	private int trafficLightID = 0;

	/// <summary>
	/// The points normal light.
	/// </summary>
	[SerializeField]
	public List<TSPointReference> pointsNormalLight =  new List<TSPointReference>();

	/// <summary>
	/// The enable light.
	/// </summary>
	public bool enableLight = true;

	/// <summary>
	/// The we have manager.
	/// </summary>
	private bool weHaveManager= false;

	/// <summary>
	/// The yellow lights stop traffic.
	/// </summary>
	public bool yellowLightsStopTraffic = true;

	/// <summary>
	/// The light range.
	/// </summary>
	public float lightRange = 500;

	/// <summary>
	/// The light to play.
	/// </summary>
	public int lightToPlay = 0;

	// Use this for initialization
	void Start () {

		trafficLightID = this.GetInstanceID();
		if (manager == null)
			manager = GameObject.FindObjectOfType(typeof(TSMainManager)) as TSMainManager;
		if (manager == null)
			weHaveManager = false;
		else weHaveManager = true;
		foreach(TSLight light in lights)
		{
			if (light.enableDisableRenderer)
			{
				if (light.lightMeshRenderer != null)
				light.lightMeshRenderer.enabled = false;
			}
			if (light.lightGameObject != null)
			{
				light.lightGameObject.SetActive(false);
			}
		}
		if (lights.Count >0 && weHaveManager) StartCoroutine(PlayLights());
		RegisterIntoLanes();
	}


	void RegisterIntoLanes()
	{
		//This is not available on Lite version, needs to keep commented
//		for (int i=0; i < pointsNormalLight.Count; i++)
//		{
//			manager.lanes[pointsNormalLight[i].lane].trafficLight = this;
//		}
	}

	/// <summary>
	/// Plays the lights.
	/// </summary>
	/// <returns>The lights.</returns>
	IEnumerator PlayLights()
	{
		while(true){
			if (lightToPlay >= lights.Count) lightToPlay = 0;
			if (lights[lightToPlay] != null)
			{
				if (!lights[lightToPlay].enableDisableRenderer){
					if (lights[lightToPlay].lightMeshRenderer != null && lights[lightToPlay].lightMeshRenderer.material != null){
						lights[lightToPlay].lightMeshRenderer.material.SetTexture(lights[lightToPlay].shaderTexturePropertyName,lights[lightToPlay].lightTexture);
					}
				}else
				{
					if (lights[lightToPlay].lightMeshRenderer != null)
						lights[lightToPlay].lightMeshRenderer.enabled = true;
				}

				if (lights[lightToPlay].lightGameObject != null)
				{
					lights[lightToPlay].lightGameObject.SetActive(true);
				}

				switch (lights[lightToPlay].lightType)
				{
				case TSTrafficLight.LightType.Yellow:
					if (yellowLightsStopTraffic)
					{
						for (int i =0; i < pointsNormalLight.Count;i++)
						{
							ChangeReservation(pointsNormalLight[i],trafficLightID,trafficLightID);
						}
					}
					break;
				case TSTrafficLight.LightType.Red:
					for (int i =0; i < pointsNormalLight.Count;i++)
					{
						ChangeReservation(pointsNormalLight[i],trafficLightID,trafficLightID);
					}
					break;
				case TSTrafficLight.LightType.Green:
					for (int i =0; i < pointsNormalLight.Count;i++)
					{
						ChangeReservation(pointsNormalLight[i],0,-1);
					}
					break;
				}
				yield return new WaitForSeconds(lights[lightToPlay].lightTime);
				if (lights[lightToPlay].enableDisableRenderer)
				{
					if(lights[lightToPlay].lightMeshRenderer != null)
						lights[lightToPlay].lightMeshRenderer.enabled = false;
				}
				if (lights[lightToPlay].lightGameObject != null)
				{
					lights[lightToPlay].lightGameObject.SetActive(false);
				}
				lightToPlay++;
			}else lightToPlay++;
		}
	}

	/// <summary>
	/// Updates the remaning green light time.
	/// </summary>
	/// <returns>The remaning green light time.</returns>
	IEnumerator UpdateRemaningGreenLightTime(List<TSPointReference> point, float time)
	{
		for (int i = 0; i < point.Count;i++)
			manager.lanes[point[i].lane].connectors[point[i].connector].remainingGreenLightTime = time;
		while (manager.lanes[point[0].lane].connectors[point[0].connector].remainingGreenLightTime > 0)
		{
			for (int i = 0; i < point.Count;i++){
				manager.lanes[point[i].lane].connectors[point[i].connector].remainingGreenLightTime -= Time.deltaTime;
				if (manager.lanes[point[i].lane].connectors[point[i].connector].remainingGreenLightTime < 0)
					manager.lanes[point[i].lane].connectors[point[i].connector].remainingGreenLightTime = 0;
			}
			yield return null;
		}

	}

	/// <summary>
	/// Changes the reservation ID.
	/// </summary>
	/// <param name="point">Point.</param>
	/// <param name="ID">I.</param>
	/// <param name="lID">L I.</param>
	void ChangeReservation(TSPointReference point, int ID, int lID)
	{
		if (point.connector == -1){
			manager.lanes[point.lane].points[point.point].reservationID = ID;
		}
		else
		{
			manager.lanes[point.lane].connectors[point.connector].points[point.point].reservationID = ID;

			if (ID ==0 && lID == -1){
				manager.lanes[point.lane].connectors[point.connector].connectorReservedByTrafficLight = false;
				manager.lanes[point.lane].connectors[point.connector].remainingGreenLightTime = -1;
			}else
				{
				manager.lanes[point.lane].connectors[point.connector].connectorReservedByTrafficLight = true;
				manager.lanes[point.lane].connectors[point.connector].remainingGreenLightTime = -1;
				manager.lanes[point.lane].connectors[point.connector].points[point.point].carwhoReserved = null;
			}

		}
	}

}
