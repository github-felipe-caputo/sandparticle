using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ParticleSystem : MonoBehaviour
{
    public Material mat;

    public ParticleEmitter partEmit;

    private Vector3[] particlePos;
    private Quaternion[] particleRot;

    public List<GameObject> particles = new List<GameObject>();
    private GameObject sphere;

    public int currentParticles = 0;

    // Public values
    public Vector3 posVar;
    public float yaw;
    public float yawVar;
    public float pitch;
    public float pitchVar;
    public int totalParticles;
    public float emissionRateSec;
    public int emitsPerRate;
    public float life;

    // Use this for initialization
    void Start() {
        // Creates and places a particle emitter at this gameobject's position
        partEmit = new ParticleEmitter(transform.position, posVar, yaw, yawVar, pitch, pitchVar, totalParticles, emissionRateSec, emitsPerRate, life);
        particlePos = new Vector3[totalParticles];
        particleRot = new Quaternion[totalParticles];
    }

    // Update is called once per frame
    void Update()
    {
        float dt = 0.003f;// Time.deltaTime;

        // Update particles
        partEmit.updateParticles(dt);

        // Get the values from the emmitter
        particlePos = partEmit.getParticlesPos();
        particleRot = partEmit.getParticlesRot();

        // update existing ones
        for (int i = 0; i < currentParticles; ++i) {
            particles[i].transform.position = particlePos[i];
            particles[i].transform.rotation = particleRot[i];
        }

        // Create new ones
        while(currentParticles < partEmit.numberAliveParticles() ) {
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.GetComponent<Renderer>().material = mat;
            //sphere.GetComponent<MeshRenderer>().material.color = new Vector4(1f, 0.2f, 0.2f, 1.0f);
            sphere.transform.position = particlePos[currentParticles];
            sphere.transform.rotation = particleRot[currentParticles];
            particles.Add(sphere);
            ++currentParticles;
        }
    }

    /* Getting screenshots to make a video
    ulong screen = 0;
    int frame = 1;
    void LateUpdate() {
        if (screen >= 10000) {
            Debug.Log(Time.frameCount + " / " + Time.time);
            #if UNITY_EDITOR
                UnityEditor.EditorApplication.isPlaying = false;
            #endif
        }

        if (frame * 0.003 > 0.016) // frame * timestep
        {
            Application.CaptureScreenshot("Screenshots/Screenshot" + screen + ".png");
            ++screen;
            frame = 1;
        }
        else {
            ++frame;
        }
    }
    */
}
