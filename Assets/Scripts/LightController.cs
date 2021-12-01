using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightController : MonoBehaviour
{
	private float newPhi = 0f;
	private float newPsi = 0f;
	private float newTheta = 0f;

	private float t0 = 0f;
	public float timeToChangeOrientationGoal = 4f; // seconds
	public float rotationSpeed = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        t0 = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
    	if (Time.time - t0 > timeToChangeOrientationGoal)
    	{
    		newPhi = Random.Range(-90, 90);
    		newPsi = Random.Range(-90, 90);
    		newTheta = Random.Range(-90, 90);
    		// Debug.Log("Changing orientation goal");
    		t0 = Time.time;
    	}
    	// Smoothly change orientation towards the goal
        transform.localRotation = Quaternion.Lerp( transform.localRotation, Quaternion.Euler(newPhi, newPsi, newTheta), Time.deltaTime*rotationSpeed);
    }
}
