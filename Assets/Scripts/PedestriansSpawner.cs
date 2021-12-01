using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PedestriansSpawner : MonoBehaviour
{
    public GameObject[] prefabs;
	public int pedestriansNumber;
	public GameObject spawnCenter;
	public float spawnRadius = 1.5f;
	public ImageSynthesis synth;

    // Start is called before the first frame update
    void Start()
    {
    	for (int i=0; i<pedestriansNumber; i++)
    	{
    		// Position
	        float newX, newY, newZ;
	        newX = Random.Range(-spawnRadius, spawnRadius);
	        newY = 0f;
	        newZ = Random.Range(-spawnRadius, spawnRadius);
	        Vector3 newPos = new Vector3(newX, newY, newZ) + spawnCenter.transform.position;
	        // Orientation
	        float newYaw = Random.Range(-90f, 90f);
	        // instantiate object
	        int prefabIndx = Random.Range(0, prefabs.Length); // choose among prefabs randomly
	        var newObj = Instantiate(prefabs[prefabIndx]);
	        // set position and orientation
	        newObj.transform.position = newPos;
	        newObj.transform.rotation = Quaternion.Euler(0, newYaw, 0);;  	
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	// in order for image synthesis script to work on updated scene
        synth.OnSceneChange();
    }
}
