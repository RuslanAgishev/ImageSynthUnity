using UnityEngine;
using System.Collections;

public class ITSWheelSize : MonoBehaviour {

	void OnDrawGizmos()
	{
		Gizmos.color = Color.yellow;
		Gizmos.DrawWireSphere(transform.position, transform.GetComponent<WheelCollider>().radius);
		
	}
}
