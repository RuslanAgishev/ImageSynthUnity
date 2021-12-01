using UnityEngine;
using System.Collections;


/// <summary>
/// TS simple car_ wheel class.  this class is responsible for rotating the visual representation of the wheel
/// when using the wheelcollider physics.
/// </summary>
public class TSSimpleCar_Wheel : MonoBehaviour {

	/// <summary>
	/// The corresponding collider.
	/// </summary>
	public WheelCollider CorrespondingCollider;

	/// <summary>
	/// The slip prefab.  This is for adding smoke when the car slides.
	/// </summary>
	public GameObject SlipPrefab;

	/// <summary>
	/// The rotation value.
	/// </summary>
	private float RotationValue = 0.0f;

	/// <summary>
	/// Cached value of the wheel transform position.
	/// </summary>
	private Vector3 pos1 = Vector3.zero;

	/// <summary>
	/// Cached value of wheelTransform.Up.
	/// </summary>
	private Vector3 up =  Vector3.zero;

	/// <summary>
	/// The suspension travel.
	/// </summary>
	private float suspensionTravel = 0f;

	/// <summary>
	/// The radius of the wheel.
	/// </summary>
	private float radius = 0f;

	/// <summary>
	/// The wheel transform.  This is a cached reference of the transform.
	/// </summary>
	private Transform wheelTransform;

	/// <summary>
	/// The actual compression of the suspension.
	/// </summary>
	private float _compression = 0f;

	/// <summary>
	/// The simple car script reference.
	/// </summary>
	private TSSimpleCar simpleCarScript ;

	/// <summary>
	/// My transform.  Cached reference of the transform.
	/// </summary>
	private Transform myTransform;

	/// <summary>
	/// The _anti roll bar force.
	/// </summary>
	private float _antiRollBarForce = 0;

	/// <summary>
	/// My body.  References to the car rigidbody
	/// </summary>
	private Rigidbody myBody;

	/// <summary>
	/// The wheel position.
	/// </summary>
	private string wheelPosition;

	private Transform myParentTransform;

	/// <summary>
	/// Gets or sets the wheel position.
	/// </summary>
	/// <value>The wheel position.</value>
	public string WheelPosition {
		get {
			return wheelPosition;
		}
		set {
			wheelPosition = value;
		}
	}

	/// <summary>
	/// Gets or sets the anti roll bar force.
	/// </summary>
	/// <value>The anti roll bar force.</value>
	public float AntiRollBarForce {
		get {
			return _antiRollBarForce;
		}
		set {
			_antiRollBarForce = value;
		}
	}

	/// <summary>
	/// Gets the compression.
	/// </summary>
	/// <value>The compression.</value>
	public float compression {
		get {
			return _compression;
		}
	}



	void Awake()
	{
		myBody = transform.parent.parent.GetComponent<Rigidbody>();
		myTransform = transform;
		myParentTransform = myTransform.parent;
		simpleCarScript = transform.root.GetComponent<TSSimpleCar>();
		wheelTransform = CorrespondingCollider.transform;
		suspensionTravel = CorrespondingCollider.suspensionDistance;
		radius = CorrespondingCollider.radius;
		if (simpleCarScript.superSimplePhysics)
		{
//			Renderer renderer = GetComponentInChildren<Renderer>();
//			SphereCollider thisCollider = renderer.gameObject.AddComponent<SphereCollider>();
//			thisCollider.sharedMaterial = transform.root.GetComponentInChildren<Collider>().sharedMaterial;
//			thisCollider.center = new Vector3(thisCollider.radius*0.70f * Mathf.Sign(-transform.parent.parent.localPosition.x),thisCollider.center.y,thisCollider.center.z);
			this.enabled = false;
		}
		else
		{
			Renderer renderer = GetComponentInChildren<Renderer>();
			Collider thisCollider = renderer.gameObject.GetComponent<Collider>();
			if (thisCollider != null)thisCollider.enabled = false;
		}
	}

	void Update () {

		if (simpleCarScript.superSimplePhysics){
			CorrespondingCollider.enabled = false;
			this.enabled = false;return;}
		pos1 = wheelTransform.position;
		up = wheelTransform.up;
		WheelHit CorrespondingGroundHit;
		bool grounded = CorrespondingCollider.GetGroundHit(out CorrespondingGroundHit );
		//Check if the wheel is grounded and set the compresion value accordingly
		if (grounded)
			_compression = 1.0f - ((Vector3.Dot(pos1 - CorrespondingGroundHit.point, up ) - radius ) / suspensionTravel);
		else _compression = suspensionTravel;

		//Set the local position of the transform
		myTransform.localPosition =  (Vector3.up * (_compression - 1.0f) * suspensionTravel);

		//Set the rotation of the transform
		myTransform.rotation = CorrespondingCollider.transform.rotation * Quaternion.Euler( RotationValue, CorrespondingCollider.steerAngle, 0 );

		//Increase the rotation value
		RotationValue += CorrespondingCollider.rpm * ( 360f/60f ) * Time.deltaTime;



		//Check if we are sliding on the sides
		if ( Mathf.Abs( CorrespondingGroundHit.sidewaysSlip ) > 2.0f ) {
			if ( SlipPrefab ) {
				Instantiate( SlipPrefab, CorrespondingGroundHit.point, Quaternion.identity );
			}
		}
		
	}

	void FixedUpdate()
	{
		if (simpleCarScript.superSimplePhysics){
			CorrespondingCollider.enabled = false;
			this.enabled = false;return;}
		myBody.AddForceAtPosition(_antiRollBarForce * myParentTransform.up,myParentTransform.position);
	}
}
