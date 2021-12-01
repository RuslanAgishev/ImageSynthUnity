using UnityEngine;
using System.Collections;

public class GUIOptions : MonoBehaviour {

	TSTrafficSpawner spawner;
	float totalCars;

	// Use this for initialization
	void Start () {
		spawner = GameObject.FindObjectOfType<TSTrafficSpawner>();
		totalCars = spawner.TrafficCars.Length;
	}
	
	void OnGUI(){
		GUI.Label(new Rect(10,Screen.height-45,350,25),"Target Amount of cars: " + spawner.amount.ToString());
		GUI.Label(new Rect(10,Screen.height-25,350,25),"Actual Amount of cars on scene: " + (totalCars - spawner.totalFarCars).ToString());
		spawner.amount = Mathf.RoundToInt( GUI.HorizontalSlider(new Rect(10,Screen.height-55,250,25),spawner.amount,0,totalCars));

		GUI.Label(new Rect(10,Screen.height-110,350,25),"Respawn Time " + spawner.respawnUpSideDownTime.ToString());
		spawner.respawnUpSideDownTime =  GUI.HorizontalSlider(new Rect(120,Screen.height-110,100,25),spawner.respawnUpSideDownTime,0f,20f);
		spawner.respawnIfUpSideDown = GUI.Toggle(new Rect(10,Screen.height-90,250,25),spawner.respawnIfUpSideDown,"Auto Respawn upside down cars?");
	}
}
