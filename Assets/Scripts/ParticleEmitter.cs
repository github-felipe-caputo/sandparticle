using UnityEngine;
using System.Collections;

public class ParticleEmitter {

    // Our Particle
    public SandParticle[] particles;

    // Emitter info
    private Vector3 pos;
    private Vector3 posVar;
    private float yaw, yawVar;
    private float pitch, pitchVar;
    private float speed, speedVar;

    //private Particle[] particles;                   // particles on the emitter
    private int totalParticles;
    private int particleCount;                      // particles emitted right now / alive
    private float emissionRateSec;                  // what is going to be the emission rate, every 0.2 secs?
    private float timeLastEmission;
    private int emitsPerRate;                       // emits per Rate, variation
    private float life, lifeVar;                    // live of particles, in seconds

    private float currentTimeEmission;

    // Constructor
    public ParticleEmitter(Vector3 newPos, Vector3 newPosVar, float newYaw, float newYawVar, float newPitch, float newPitchVar, 
        int newTotalParticles, float newEmissionRateSec, int newEmitsPerRate, float newLife) {
        // Position of the emitter
        pos = newPos;
        posVar = newPosVar;

        // Values that will affect direction
        yaw = newYaw;
        yawVar = newYawVar;
        pitch = newPitch;
        pitchVar = newPitchVar;

        // How many particles can be alive at the same time
        totalParticles = newTotalParticles;
        particles = new SandParticle[totalParticles];

        // The rate (in secs) it will emit new particles
        emissionRateSec = newEmissionRateSec;

        // How many particles will emit per rate
        emitsPerRate = newEmitsPerRate;
        
        // How long will each particle last
        life = newLife;

        timeLastEmission = 0;
        particleCount = 0;
    }

    // Emit will return a particle list, which will be rendered
    public void updateParticles(float dt) {
        int currentEmission = 0;
        currentTimeEmission += dt;

        // add particle
        // if current number of particle is lower than total we can add particles
        // since we work on emissions per second, one second need to past after timeLastEmission
        if (particleCount < totalParticles && currentTimeEmission > emissionRateSec) {
            while (currentEmission < emitsPerRate)
            {
                particles[particleCount] = new SandParticle(pos + new Vector3(Random.Range(-1.0f, 1.0f) * posVar.x, Random.Range(-1.0f, 1.0f) * posVar.y, Random.Range(-1.0f, 1.0f) * posVar.z),
                                                            20 * fromRotationToDirection(yaw + Random.Range(-1.0f, 1.0f) * yawVar, pitch + Random.Range(-1.0f, 1.0f) * pitchVar));
                ++currentEmission;
                ++particleCount;
            }
            currentTimeEmission = 0;
        }
        
        // update particles
        for (int i = 0; i < particleCount; i++) {
                particles[i].updateParticle(dt, particles, particleCount);
        }
    }

    public int numberAliveParticles() {
        return particleCount;
    }

    public Vector3[] getParticlesPos() {
        Vector3[] positions = new Vector3[particleCount];

        for (int i = 0; i < particleCount; ++i) {
            positions[i] = particles[i].position;
        }

        return positions;
    }

    public Quaternion[] getParticlesRot() {
        Quaternion[] rotations = new Quaternion[particleCount];

        for (int i = 0; i < particleCount; ++i) {
            rotations[i] = particles[i].rotation;
        }

        return rotations;
    }

    private Vector3 fromRotationToDirection(float yaw, float pitch) {
        float nYaw = yaw * Mathf.Deg2Rad;
        float nPitch = pitch * Mathf.Deg2Rad;
        return new Vector3(-Mathf.Sin(nYaw) * Mathf.Cos(nPitch), Mathf.Sin(nPitch), Mathf.Cos(nPitch) * Mathf.Cos(nYaw));
    }
}
