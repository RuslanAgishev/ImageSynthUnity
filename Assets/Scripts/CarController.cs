using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;


public class CarController : MonoBehaviour
{
	private Camera mainCamera;
	private NavMeshAgent agent;
    private Vector3 destination;
    private bool goalReached = false;
    private Vector3 goal;
    private float t0;

    public bool DefineGoalWithMouse = false;
    public float goalTol = 1f; // goal tolerance: waypoint si considered as reached
    public float goalWaitTime = 3f; // [sec], time to wait before switching to next waypoint
    public float sampleRadiusLateral = 10f; // [m], waypoints sampling radius along lateral direction
    public float sampleRadiusForward = 50f; // [m], waypoints sampling radius along forward direction

    // Start is called before the first frame update
    void Start()
    {
        mainCamera = Camera.main;
        agent = GetComponent<NavMeshAgent>();
        goal = transform.position;
        t0 = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        // choose goal with mouse pointer
        if (DefineGoalWithMouse)
        {
            ChooseGoalWithMouse();
        }
        else
        {
            ChooseGoal(goalTol, goalWaitTime, sampleRadiusLateral, sampleRadiusForward);
        }
    }

    void ChooseGoal(float dist_eps=1f, float time_eps=3f, float Rx=50f, float Rz=50f)
    {
        float distance = Vector3.Distance(goal, transform.position);
        // Debug.Log("Time elappsed: " + (Time.time - t0));
        // Debug.Log("Distance to goal: " + distance);
        if (distance < dist_eps || Time.time - t0 > time_eps)
        {
            // Debug.Log("Goal is reached");
            float x = transform.position.x;
            float y = transform.position.y;
            float z = transform.position.z;
            goal = new Vector3(Random.Range(x-Rx, x+Rx), y, Random.Range(z, z+Rz));
            // set goal for agent to go
            agent.SetDestination(goal);
            t0 = Time.time;
        }
    }

    void ChooseGoalWithMouse()
    {
        if (Input.GetMouseButtonDown(0))
            {
                RaycastHit hit;
                if (Physics.Raycast(mainCamera.ScreenPointToRay(Input.mousePosition), out hit))
                {
                    goal = hit.point;
                    // set goal for agent to go
                    agent.SetDestination(goal);
                }
            }
    }

    float Norm(Vector3 v)
    {
        return Mathf.Sqrt(v[0]*v[0] + v[1]*v[1]+ v[2]*v[2]);
    }
}
