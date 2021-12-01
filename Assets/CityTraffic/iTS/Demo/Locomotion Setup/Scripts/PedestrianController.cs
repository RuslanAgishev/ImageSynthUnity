using UnityEngine;
using System.Collections;

public class PedestrianController : MonoBehaviour {

	protected Animator animator;
	
	private float speed = 0;
	public float direction = 0;
	private Locomotion locomotion = null;
	private Rigidbody body;
	// Use this for initialization
	void Start () 
	{
		body = GetComponent<Rigidbody>();
		animator = GetComponent<Animator>();
		locomotion = new Locomotion(animator);

		TSTrafficAI ai = GetComponent<TSTrafficAI>();
		ai.OnUpdateAI = OnAIUpdate;
		ai.UpdateCarSpeed = UpdateSpeed;

	}


	void UpdateSpeed(out float carSpeed)
	{
		carSpeed = body.velocity.z;
	}
	
	void OnAIUpdate(float steering, float brake, float throttle, bool isUpSideDown ){
		speed = Mathf.Clamp01(throttle - brake);
		direction = steering;
		
	}


	void Update () 
	{
	
		if (animator)
		{
			if (body.constraints != RigidbodyConstraints.FreezeRotation)body.constraints = RigidbodyConstraints.FreezeRotation;
			locomotion.Do(speed , 45* direction);
		}		
	}
}
