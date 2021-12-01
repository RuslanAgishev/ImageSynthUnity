using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TSSimpleCar : MonoBehaviour {

	/// <summary>
	/// The front wheels.  This is used for the normal wheelcollider car physics, here goes all the wheels that
	/// could be steer
	/// </summary>
	public WheelCollider[] FrontWheels = new WheelCollider[0];

	/// <summary>
	/// The back wheels.   This is used for the normal wheelcollider car physics, here goes all the wheels that
	/// would be powered by the engine
	/// </summary>
	public WheelCollider[] BackWheels = new WheelCollider[0];

	/// <summary>
	/// The aditional brake wheels.  Put here any other wheels on this car that would only be used for braking, like i.e.
	/// the wheels of a trailer.
	/// </summary>
	public WheelCollider[] aditionalBrakeWheels = new WheelCollider[0];

	/// <summary>
	/// The player controlled.
	/// </summary>
	public bool playerControlled = false;

	public bool engineAudioEnabled = true;

	/// <summary>
	/// The gear ratios.
	/// </summary>
	public float[] GearRatio = {3.5f,2.5f,2f,1.5f,1f};

	/// <summary>
	/// The current gear.
	/// </summary>
	public int CurrentGear;
	
	/// <summary>
	/// The engine torque.  The actual torque the engine would transfer to the powered wheels.
	/// </summary>
	public float EngineTorque = 600.0f;

	/// <summary>
	/// The max engine RPM.
	/// </summary>
	public float  MaxEngineRPM = 3000.0f;

	/// <summary>
	/// The minimum engine RPM.
	/// </summary>
	public float  MinEngineRPM = 1000.0f;

	/// <summary>
	/// The max acceleration.  This would limit the cars acceleration, lower values makes the car accelerate slower.
	/// </summary>
	public float maxAcceleration = 5f;

	//*****************************TO BE REPLACED********************************************
	//This would be replaced with the anti-rollbar script, for more real behavior of the cars.
//	public float mass_center = 0.025f;
	//*****************************TO BE REPLACED********************************************

	/// <summary>
	/// The brake torque.
	/// </summary>
	public float brakeTorque = 400;

	/// <summary>
	/// The brake ligths.
	/// </summary>
	public Renderer[] brakeLigths;

	/// <summary>
	/// The brake intensity rate.  Used only if enableDisableBrakeLights is set to false, otherwise the brake lights
	/// renderers are just enabled and disabled.
	/// </summary>
	public float brakeIntensityRate = 0.2f;

	/// <summary>
	/// The enable disable brake lights.
	/// </summary>
	public bool enableDisableBrakeLights = false;

	/// <summary>
	/// The brake lights shader.
	/// </summary>
	public Shader brakeLightsShader;

	/// <summary>
	/// The name of the property.  This is the property name of the brake lights of the car, that would be used to set
	/// the intensity of the shader according to the brake input.
	/// </summary>
	public string propertyName = "_Intensity";

	/// <summary>
	/// The super simple physics.
	/// </summary>
	public bool superSimplePhysics = false;


	/*EXPERIMENTAL, THIS IS FOR ATTEMPTING TO SWITCH BETTWEN THE TYPES OF PHYSICS
	/// <summary>
	/// The autoswitch normal to super simple physics.
	/// </summary>
	//public bool AutoswitchNormalToSuperSimplePhysics = false;

	/// <summary>
	/// The super simple physics tire colliders.  Put here the Sphere colliders that would be like the tires when the car
	/// is on super simple physics, this is to be able to disable when auto switch from super simple physics is on.
	/// </summary>
	//public Collider[] superSimplePhysicsTireColliders;
	*/

	/// <summary>
	/// The turn speed.
	/// </summary>
	public float turnSpeed = 3f;

	/// <summary>
	/// The car grip.
	/// </summary>
	public float carGrip = 70f;

	/// <summary>
	/// The max speed.
	/// </summary>
	public float maxSpeed = 50f;

	/// <summary>
	/// The max speed to turn.
	/// </summary>
	public float maxSpeedToTurn = 0.2f;

	/// <summary>
	/// Up side down.
	/// </summary>
	private bool upSideDown = false;

	/// <summary>
	/// The crashed.  Enable this by script to avoid this car to continue trying to move if the collision was big
	/// enough to totally damage the car.
	/// </summary>
	[HideInInspector]
	public bool crashed = false;

	/// <summary>
	/// The car can crash.  This should be enabled if this car should no longer be able to move if it gets crashed.
	/// </summary>
	public bool carCanCrash = false;

	/// <summary>
	/// The crashed smokes.  Additional and optional particles effects to add to the cars, and that would be activated
	/// if the crashed bool is set to true.
	/// </summary>
	public ParticleSystem[] crashedSmokes;

	/// <summary>
	/// The minimum relative speed when this car have a collision to enable crash and the crashed particles system.
	/// </summary>
	public float minSpdForCrash = 10f;

	/// <summary>
	/// The turn right light.  This is for placing the game object that represents the right turning light;
	/// </summary>
	public GameObject turnRightLight;

	/// <summary>
	/// The turn left light.  This is for placing the game object that represents the left turning light;
	/// </summary>
	public GameObject turnLeftLight;

	/// <summary>
	/// The center of mass.
	/// </summary>
	public Transform CoM;

	/// <summary>
	/// The engine RPM.
	/// </summary>
	private float  EngineRPM = 0.0f;

	/// <summary>
	/// The index of the brake lights.
	/// </summary>
//	private int[] brakeLightsIndex;

	/// <summary>
	/// The brake.
	/// </summary>
	private float brake = 0f;

	/// <summary>
	/// The brake intensity.
	/// </summary>
//	private float brakeIntensity = 0f;

	/// <summary>
	/// The RPM.  this is the RPM from the wheels.
	/// </summary>
	private float RPM = 0;

	/// <summary>
	/// The body.
	/// </summary>
	private Rigidbody body;

	/// <summary>
	/// The bodies.
	/// </summary>
	private Rigidbody[] bodies;

	/// <summary>
	/// The steering.
	/// </summary>
	private float steering = 0f;

	/// <summary>
	/// My transform.  Cached reference of the tansform.
	/// </summary>
	private Transform myTransform;

	/// <summary>
	/// The traffic AI.  Reference to the class TSTrafficAI.
	/// </summary>
	private TSTrafficAI trafficAI;

	//Variables to control the actual acceleration speed of the car
	/// <summary>
	/// The last speed.
	/// </summary>
	private float lastSpeed;

	/// <summary>
	/// The speed.
	/// </summary>
	private float speed;

	/// <summary>
	/// My accel.
	/// </summary>
	private float myAccel;

	/// <summary>
	/// The blinking.  This is true if any of the turning lights is blinking.
	/// </summary>
	private bool blinking = false;

	//Super simple car physics variables
	/// <summary>
	/// The front wheels.
	/// </summary>
	private Transform[] frontWheels;

	/// <summary>
	/// The rear wheels.
	/// </summary>
	private Transform[] rearWheels;

	/// <summary>
	/// My speed.
	/// </summary>
	private float mySpeed = 0f;

	/// <summary>
	/// The velo.
	/// </summary>
	private Vector3 velo = Vector3.zero;

	/// <summary>
	/// The temp vec.
	/// </summary>
	private Vector3 tempVec;

	/// <summary>
	/// The flat velo.
	/// </summary>
	private Vector3 flatVelo;

	/// <summary>
	/// The flat dir.
	/// </summary>
	private Vector3 flatDir;

	/// <summary>
	/// The slide speed.
	/// </summary>
	private float slideSpeed = 0f;

	/// <summary>
	/// The engine force.
	/// </summary>
	private Vector3 engineForce = Vector3.zero;

	/// <summary>
	/// The actual grip.
	/// </summary>
	private float actualGrip = 0f;

	/// <summary>
	/// The impulse.
	/// </summary>
	private Vector3 impulse = Vector3.zero;

	/// <summary>
	/// The dir.
	/// </summary>
	private Vector3 dir;

	private AudioSource audioSource;

	static WaitForSeconds waitforseconds = new WaitForSeconds(0.2f);

	/// <summary>
	/// All wheels.
	/// </summary>
	/*PART OF EXPERIMENTAL CODE TO AUTO SWITCH FROM AND TO SUPER SIMPLE PHYSICS*/
//	private TSSimpleCar_Wheel[] allWheels;

	void Awake()
	{
		//Get the cached references
		myTransform = transform;
		body = GetComponent<Rigidbody>();
		bodies = GetComponentsInChildren<Rigidbody>();
		trafficAI = GetComponent<TSTrafficAI>();
		audioSource = GetComponent<AudioSource>();
		if (audioSource ==null)engineAudioEnabled = false;

		/*PART OF EXPERIMENTAL CODE TO AUTO SWITCH FROM AND TO SUPER SIMPLE PHYSICS*/
//		allWheels = GetComponentsInChildren<TSSimpleCar_Wheel>();
	}

	void Start () {


		//Cache the waitfor seconds for the blinking light code


		/*Register the AI script callbacks*/
		if (trafficAI!= null){
			trafficAI.OnUpdateAI += OnAIUpdate;
			trafficAI.UpdateCarSpeed += UpdateCarSpeed;
			trafficAI.OnTurnLeft += OnTurnLeft;
			trafficAI.OnTurnRight += OnTurnRight;

			/*EXPERIMENTAL CODE, FOR ATTEMPTING TO AUTO SWITCH BETWEEN THE 2 TYPES OF PHYSICS
			trafficAI.OnCloserRange = OnCloserRange;
			trafficAI.OnFarRange = OnFarRange;*/
		}

		/*Set the turning lights off at Start*/
		if (turnLeftLight != null)
			turnLeftLight.SetActive(false);
		if (turnRightLight != null)
			turnRightLight.SetActive(false);

		//gets a default shader if the one especified is null
		if (brakeLightsShader == null) brakeLightsShader = Shader.Find("Car/LightsEmmissive");
	

		//Super simple physics initialization
		if (superSimplePhysics)
		{
			frontWheels = new Transform[FrontWheels.Length];
			rearWheels = new Transform[BackWheels.Length];
			for (int i = 0; i < FrontWheels.Length;i++)
			{
				frontWheels[i] = FrontWheels[i].transform;
				FrontWheels[i].enabled = false;
			}
			for (int i = 0; i < BackWheels.Length;i++)
			{
				rearWheels[i] = BackWheels[i].transform;
				BackWheels[i].enabled = false;
			}
			for (int i =0; i < bodies.Length;i++){
				if (CoM !=null && i ==0)
					bodies[i].centerOfMass = new Vector3(CoM.localPosition.x,-2,CoM.localPosition.z);
				else
					bodies[i].centerOfMass = new Vector3(bodies[i].centerOfMass.x,-2,bodies[i].centerOfMass.z);
			}
		}
		//Normal wheelcollider initialization
		else
		{

			if (CoM != null)
			{
				for (int i =0; i < bodies.Length;i++)
					if (i ==0)
						bodies[i].centerOfMass = new Vector3(CoM.localPosition.x * transform.localScale.x, CoM.localPosition.y * transform.localScale.y, CoM.localPosition.z * transform.localScale.z);	//new Vector3(bodies[i].centerOfMass.x,-1,bodies[i].centerOfMass.z);
				body.centerOfMass = new Vector3(CoM.localPosition.x * transform.localScale.x, CoM.localPosition.y * transform.localScale.y, CoM.localPosition.z * transform.localScale.z);	
//				Debug.Log ("Set center of mass for car" + " " +myTransform.root.name);
			}
		}
	}

	void OnEnable()
	{
		if (CoM != null)
		{
			for (int i =0; i < bodies.Length;i++)
				if (i ==0)
					bodies[i].centerOfMass = new Vector3(CoM.localPosition.x * transform.localScale.x, CoM.localPosition.y * transform.localScale.y, CoM.localPosition.z * transform.localScale.z);	//new Vector3(bodies[i].centerOfMass.x,-1,bodies[i].centerOfMass.z);
			body.centerOfMass = new Vector3(CoM.localPosition.x * transform.localScale.x, CoM.localPosition.y * transform.localScale.y, CoM.localPosition.z * transform.localScale.z);	
			//				Debug.Log ("Set center of mass for car" + " " +myTransform.root.name);
		}
		crashed = false;
	}

	/// <summary>
	/// Raises the turn right event.
	/// </summary>
	/// <param name="isTurning">If set to <c>true</c> is turning.</param>
	void OnTurnRight(bool isTurning)
	{
		if (turnRightLight != null){
			if (isTurning)
				StartCoroutine(LightBlinking(turnRightLight));
			else{
				blinking = false;
				turnRightLight.SetActive(false);
			}
		}
	}

	/// <summary>
	/// Raises the turn left event.
	/// </summary>
	/// <param name="isTurning">If set to <c>true</c> is turning.</param>
	void OnTurnLeft(bool isTurning)
	{
		if (turnLeftLight != null){
			if (isTurning)
				StartCoroutine(LightBlinking(turnLeftLight));
			else{
				blinking = false;
				turnLeftLight.SetActive(false);
			}
		}
	}




	/// <summary>
	/// Makes a light start blinking.
	/// </summary>
	/// <returns>The blinking.</returns>
	/// <param name="blinkingLight">Blinking light.</param>
	/// <param name="blinkingTiming">Blinking timing.</param>
	IEnumerator LightBlinking(GameObject blinkingLight)
	{
		blinking  = true;
		while(blinking)
		{
			if (blinkingLight.activeSelf)
				blinkingLight.SetActive(false);
			else
				blinkingLight.SetActive(true);
			yield return waitforseconds;

		}
		blinkingLight.SetActive(false);
	}


	void Update () {
		if (!crashed){
			if (superSimplePhysics && !upSideDown)
			{

				// up to twice it's pitch, where it will suddenly drop when it switches gears.
				for (int i = 0; i < bodies.Length;i++){
					float mass_center = Mathf.Clamp((((bodies[i].velocity.magnitude - 10) / 35f)),-1.00f,2f);
					bodies[i].centerOfMass = new Vector3(bodies[i].centerOfMass.x, -mass_center,bodies[i].centerOfMass.z);
					body.centerOfMass = new Vector3(bodies[i].centerOfMass.x, -mass_center,bodies[i].centerOfMass.z);
				}

				// set the audio pitch to the percentage of RPM to the maximum RPM plus one, this makes the sound play
				if (engineAudioEnabled){
					audioSource.pitch = Mathf.Abs(mySpeed / maxSpeed) + 0.75f ;
					// this line is just to ensure that the pitch does not reach a value higher than is desired.
					if ( audioSource.pitch > 2.0f ) {
						audioSource.pitch = 2.0f;
					}
				}
				//Update de brake lights effect when braking
				BrakeLight (brake);
			}
			else if(!superSimplePhysics)
			{

				if (playerControlled)
				{
					this.steering = Input.GetAxis("Horizontal");
					float motorTorque =EngineTorque  * Input.GetAxis("Vertical");
					float maxSteering = Mathf.Max(1-mySpeed/maxSpeed,0.05f);
					float steerAngle = 55f* Mathf.Clamp(steering,-1f * maxSteering,1f * maxSteering);
					float brakeT = 0;//brakeTorque * -Mathf.Clamp(Input.GetAxis("Vertical"),-1,0);

					for (int i = 0; i < FrontWheels.Length;i++)
					{
						FrontWheels[i].steerAngle = steerAngle;
						FrontWheels[i].brakeTorque = brakeT;
					}
					
					for (int i = 0; i < BackWheels.Length;i++)
					{
						if (mySpeed < maxSpeed)
							BackWheels[i].motorTorque = motorTorque/(BackWheels.Length/2f);
						else BackWheels[i].motorTorque= -EngineTorque/5f;
						BackWheels[i].brakeTorque = brakeT;
					}
					for (int i = 0; i < aditionalBrakeWheels.Length; i++) {
						aditionalBrakeWheels[i].brakeTorque = brakeT;
					}
					if (brakeT !=0) 
					{
						this.brake+=1f;
					}
					else{
						this.brake-=0.05f;
					}
					this.brake = Mathf.Clamp01(this.brake);
				}

				/*To be replaced This code*/
				// Clamping the minimum and maximum position of the center of mass provides stability at high speeds
				// This also allows stationary cars to be bowled over/roll on uneven terrain
//				for (int i = 0; i < bodies.Length;i++){
//					mass_center = Mathf.Clamp((0.3f+(bodies[i].velocity.magnitude / 35f)),0.02f,1.2f);
//					bodies[i].centerOfMass = new Vector3(bodies[i].centerOfMass.x, -mass_center,bodies[i].centerOfMass.z);
//				body.centerOfMass = new Vector3(CoM.localPosition.x * transform.localScale.x, CoM.localPosition.y * transform.localScale.y, CoM.localPosition.z * transform.localScale.z);	
//				}
				
				// Compute the engine RPM based on the average RPM of the two wheels, then call the shift gear function
				for (int i=0; i < BackWheels.Length;i++)
				{
					RPM += BackWheels[i].rpm;
				}
				RPM/=BackWheels.Length;
				EngineRPM = RPM * GearRatio[CurrentGear];
				ShiftGears();
				
				// set the audio pitch to the percentage of RPM to the maximum RPM plus one, this makes the sound play
				// up to twice it's pitch, where it will suddenly drop when it switches gears.
				if (engineAudioEnabled){
					audioSource.pitch = Mathf.Abs(EngineRPM / MaxEngineRPM) + 0.75f ;
					// this line is just to ensure that the pitch does not reach a value higher than is desired.
					if ( audioSource.pitch > 2.0f ) {
						audioSource.pitch = 2.0f;
					}
				}

				//Update de brake lights effect when braking
				if (FrontWheels!=null && FrontWheels.Length>0)
					BrakeLight (FrontWheels [0].brakeTorque);

			}
		}
	}

	/// <summary>
	/// Updates the Brake the lights state.
	/// </summary>
	/// <param name="brakeInput">Brake input.</param>
	public void BrakeLight(float brakeInput)
	{
		if (enableDisableBrakeLights){
			if (brakeLigths != null){
				for (int i =0; i < brakeLigths.Length;i++)
				{
					if (brake !=0 && brakeLigths[i].enabled == false) 
					{
						brakeLigths[i].enabled = true;
					}
					else if (brake ==0 && brakeLigths[i].enabled == true){
						brakeLigths[i].enabled = false;
					}
				}
			}
		}
		/*NOT USED CODE  HAVING THIS IS BAD FOR PERFORMANCE*/
//		else{
//
//			int x = 0;
//			if (brakeInput >0)//< 0.3f && brake1 > 0.0f)
//				brakeIntensity += brakeIntensityRate;
//			else if (brakeInput == 0.0f)
//				brakeIntensity -= brakeIntensityRate;
//			
//			brakeIntensity = Mathf.Clamp(brakeIntensity,0,5);
//			
//			if (brakeIntensity >0){
//				if (brakeLigths.Length != 0 )
//				{
//					foreach(Renderer brakelight in brakeLigths)
//					{	
//						if (brakelight && brakelight.materials.Length !=0){
//							if (brakelight.materials[brakeLightsIndex[x]]){
//								brakelight.materials[brakeLightsIndex[x]].SetFloat(propertyName, brakeIntensity);
//							}
//						}
//						x++;
//					}			
//				}
//			}
//			else if (brakeIntensity == 0.0f)
//			{
//				x=0;
//				if (brakeLigths.Length != 0 )
//				{
//					foreach(Renderer brakelight in brakeLigths)
//					{		
//						if (brakelight && brakelight.materials.Length !=0){
//							if (brakelight.materials[brakeLightsIndex[x]]){
//								brakelight.materials[brakeLightsIndex[x]].SetFloat(propertyName, brakeIntensity);
//							}
//						}
//						x++;
//					}
//				}
//			}
//		}
	}


	Vector3 rotationAmount;
	float rotation;
	/// <summary>
	/// Rotates the visual wheels.  This is part of the super simple car physics
	/// </summary>
	/// <param name="steering">Steering.</param>
	void RotateVisualWheels(float steering)
	{
		float rotationValue = (mySpeed * 1.6f * Time.deltaTime) ;
		rotation += rotationValue;
		rotationAmount = Vector3.right * rotationValue * Mathf.Rad2Deg;

		for (int i = 0; i < frontWheels.Length;i++)
		{
//			frontWheels[i].Rotate(rotationValue,0,0);
//			Vector3 tempAngles = frontWheels[i].localEulerAngles;
//			tempAngles.y = steering;
//			tempAngles.x += rotationValue;
			frontWheels[i].localEulerAngles = new Vector3(rotation * Mathf.Rad2Deg, steering,0);// tempAngles;

		}
		for (int i = 0; i < rearWheels.Length;i++)
		{
			rearWheels[i].Rotate(rotationAmount);
		}
	}

	/// <summary>
	/// Updte the Super simple physics state.
	/// </summary>
	/// <param name="throttle">Throttle.</param>
	/// <param name="steering">Steering.</param>
	void SuperSimplePhysicsUpdate(float throttle, float steering)
	{
		velo = body.velocity;
		tempVec = new Vector3(velo.x,0,velo.z);
		flatVelo = tempVec;
		dir = myTransform.TransformDirection(Vector3.forward);
		tempVec = new Vector3(dir.x,0,dir.z);
		flatDir = Vector3.Normalize(tempVec);
		slideSpeed = Vector3.Dot(myTransform.right,flatVelo);
		mySpeed = flatVelo.magnitude;
		engineForce = ((flatDir) * (EngineTorque * 10 * throttle)* body.mass);
		actualGrip = Mathf.Lerp(100,carGrip, mySpeed * 0.02f);
		impulse = myTransform.right * (-slideSpeed * body.mass * Mathf.Abs(Physics.gravity.y) * actualGrip) ;
	}

	void FixedUpdate()
	{
		if (!crashed && superSimplePhysics && !upSideDown)
		{
			float timeStepFactor = (0.02f/Time.fixedDeltaTime);
			if (mySpeed < maxSpeed)
			{
				if (frontWheels != null && frontWheels.Length >0)
					body.AddForceAtPosition(engineForce * Time.deltaTime*timeStepFactor,new Vector3(myTransform.position.x, FrontWheels[0].transform.position.y, myTransform.position.z));
				else
					body.AddForce(engineForce * Time.deltaTime*timeStepFactor);
			}
			if (mySpeed > maxSpeedToTurn)
			{
				Quaternion deltaRot = Quaternion.Euler(new Vector3(0,turnSpeed * steering ,0));
				body.MoveRotation(body.rotation * deltaRot);
			}
			if (brake >0 && body.velocity.sqrMagnitude > 0)
			{
				if (frontWheels != null && frontWheels.Length >0)
					body.AddForceAtPosition((-(body.velocity.normalized)) * brakeTorque * brake * Time.deltaTime*timeStepFactor,new Vector3(myTransform.position.x, FrontWheels[0].transform.position.y, myTransform.position.z));
				else
					body.AddForce(Mathf.Sign(localspeed.z) * (-(flatDir)) * brakeTorque * 10 * body.mass * brake * Time.deltaTime*timeStepFactor);
			}
			body.AddForce(impulse * Time.deltaTime*timeStepFactor);
		}else mySpeed = body.velocity.magnitude;
		myAccel = ( speed-lastSpeed)/Time.deltaTime;
		if (myAccel > maxAcceleration)body.drag +=0.1f;
		else if(body.drag >0.1f)body.drag -=0.1f;
		if (body.drag < speed/ 125f)body.drag = speed/ 125f;
		lastSpeed = speed;
	}


	/// <summary>
	/// Raises the AI update event.
	/// </summary>
	/// <param name="steering">Steering.</param>
	/// <param name="brake">Brake.</param>
	/// <param name="throttle">Throttle.</param>
	public void OnAIUpdate(float steering, float brake, float throttle, bool isUpSideDown )
	{ 
		upSideDown = isUpSideDown;
		this.steering = steering;
		float steerAngle = 55f* Mathf.Clamp(steering,-1f,1f);
		float brakeT = brakeTorque * brake;
		float motorTorque =EngineTorque  * (throttle);
		if (motorTorque <0)motorTorque = 0;
		if (crashed) {
			steerAngle = 0;
			brakeT = brakeTorque * 1f;
			motorTorque = 0;
			throttle = 0;
			steering = 0;
		}
		if (superSimplePhysics)
		{
			RotateVisualWheels(steerAngle);
			SuperSimplePhysicsUpdate(throttle, steering);
		}
		else{

			for (int i = 0; i < FrontWheels.Length;i++)
			{
				FrontWheels[i].steerAngle = steerAngle;
				FrontWheels[i].brakeTorque = brakeT;
			}
			
			for (int i = 0; i < BackWheels.Length;i++)
			{
				if (mySpeed < maxSpeed)
					BackWheels[i].motorTorque = motorTorque/(BackWheels.Length/2f);
				BackWheels[i].brakeTorque = brakeT;
			}
			for (int i = 0; i < aditionalBrakeWheels.Length; i++) {
				aditionalBrakeWheels[i].brakeTorque = brakeT;
			}
		}


		if (brakeT !=0) 
		{
			this.brake+=1f;
		}
		else{
			this.brake-=0.05f;
		}
		this.brake = Mathf.Clamp01(this.brake);
		
	}

	/*EXPERIMENTAL CODE FOR ATTEMPTING TO AUTO SWITCH BETWEEN PHYSICS
	 * void OnCloserRange()
	{
		//Enable the normal physics
		if (!superSimplePhysics || !AutoswitchNormalToSuperSimplePhysics)return;
		superSimplePhysics = false;
		for (int i =0; i < BackWheels.Length;i++)
		{
			BackWheels[i].enabled = true;
			rearWheels[i].localRotation = Quaternion.identity;
		}
		for (int i =0; i < FrontWheels.Length;i++)
		{
			FrontWheels[i].enabled = true;
			frontWheels[i].localRotation = Quaternion.identity;
		}
		for (int i =0; i < allWheels.Length;i++)
		{
			allWheels[i].enabled = true;
		}
		for (int i = 0; i < superSimplePhysicsTireColliders.Length;i++)
		{
			superSimplePhysicsTireColliders[i].enabled = false;
		}
	}

	void OnFarRange()
	{
		//Disable the normal physics and enable super simple physics
		if (!AutoswitchNormalToSuperSimplePhysics)return;
		superSimplePhysics = true;
		for (int i = 0; i < superSimplePhysicsTireColliders.Length;i++)
		{
			superSimplePhysicsTireColliders[i].enabled = true;
		}
	}*/

	Vector3 localspeed;
	/// <summary>
	/// Updates the car speed.
	/// </summary>
	/// <param name="carSpeed">Car speed.</param>
	public void UpdateCarSpeed( out float carSpeed)
	{
			localspeed = myTransform.InverseTransformDirection(body.velocity);
			speed = carSpeed = localspeed.magnitude;// Mathf.Abs(myTransform.InverseTransformDirection(body.velocity).z);
	}

	/// <summary>
	/// Shifts the gears.
	/// </summary>
	void ShiftGears() {
		// this funciton shifts the gears of the vehcile, it loops through all the gears, checking which will make
		// the engine RPM fall within the desired range. The gear is then set to this "appropriate" value.
		int AppropriateGear = CurrentGear;
		if ( EngineRPM >= MaxEngineRPM ) {
			for ( int i = 0; i < GearRatio.Length; i ++ ) {
				if ( RPM * GearRatio[i]   < MaxEngineRPM ) {
					AppropriateGear = i;
					break;
				}
			}
			CurrentGear = AppropriateGear;
		}
		if ( EngineRPM <= MinEngineRPM  ) {
			AppropriateGear = CurrentGear;
			
			for ( int j = GearRatio.Length-1; j >= 0; j -- ) {
				if ( RPM * GearRatio[j]  > MinEngineRPM ) {
					AppropriateGear = j;
					break;
				}
			}
			CurrentGear = AppropriateGear;
		}
	}


	/// <summary>
	/// Gets the brake lights.
	/// </summary>
	/// <returns>The brake lights.</returns>
	int[] getBrakeLights()
	{
		int[] tempArray;
		int x = 0;
		tempArray = new int[brakeLigths.Length];
		x = 0;
		if (brakeLigths.Length != 0)
		{
			foreach (Renderer brakeLight in brakeLigths)
			{
				int i = 0;
				if (brakeLight!=null){
					foreach (Material brakeLightMaterial in brakeLight.sharedMaterials)
					{
						if (brakeLightMaterial != null)
							if (brakeLightMaterial.shader == brakeLightsShader )		
								tempArray[x] = i;
						i++;
					}
				}
				x++;
			}
		}
		return tempArray;
	}

	/// <summary>
	/// Activates the crashed smoke.
	/// </summary>
	public void ActivatecrashedSmoke()
	{
		for (int i = 0; i < crashedSmokes.Length; i++)
		{
			crashedSmokes[i].enableEmission = true;
			crashedSmokes[i].Play(true);
		}
	}

	void OnCollisionEnter(Collision col)
	{

		if (carCanCrash && col.relativeVelocity.magnitude > minSpdForCrash){
			crashed = true;
			if (trafficAI != null)
				trafficAI.crashed = true;
			ActivatecrashedSmoke();
		}
	}


}
