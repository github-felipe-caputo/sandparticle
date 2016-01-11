using UnityEngine;
using System.Collections;

public class SandParticle {

    // pos and rotation
    public Vector3 position;
    public Quaternion rotation;

    // Constant Values
    private float mass = 0.15f;
    private float radius = 0.5f;

    // Primary
    public Vector3 linearVelocity;
    public Vector3 linearAcceleration;

    public Vector3 angVelocity;
    public Quaternion angVelocityQua;

    // Secondary
    public Vector3 linearMomentum;
    private Vector3 angMomentum;

    // Inertia
    public Vector3 inertiaTensor;

    // Forces
    public Vector3 forceHit;
    private Vector3 forceGravity;
    private Vector3 forceTotal;

    private Vector3 Fl; // linear force
    private Vector3 Ft; // torque force

    // Rotation flag
    public bool naturalRoll = false;

    // Colision
    public bool floorCollision = false;
    public Vector3 penaltyFl; // penalty force linear
    public Vector3 penaltyFt; // penalty force torque

    // testing stuff
    private bool applyForce;

    // Runge Kutta variables
    class Derivative
    {
        public Vector3 dx; // derivative of position (velocity)
        public Vector3 dm; // derivative of linear momentum (linear force)
        public Vector3 dam; // derivative of angular momentum (torque)
    }
    Derivative a, b, c, d;


    public SandParticle(Vector3 startPosition, Vector3 startForce) {
        // torque, angular stuff
        float inertia = (2.0f / 5.0f) * mass * Mathf.Pow(radius, 2);
        inertiaTensor = new Vector3(inertia, inertia, inertia); // sphere is the same

        applyForce = true;
        forceHit = startForce;
        position = startPosition;
        rotation = new Quaternion(1,1,1,1);
    }

    public void updateParticle(float dt, SandParticle[] otherParticles, int aliveParticles) {
        // Update forces
        if (applyForce) {
            Fl = forceHit;
            applyForce = false;
        } else {
            Fl.Set(0, 0, 0);
        }

        // Integrate Velocities (Runge Kutta)
        a = evaluate(position, linearMomentum, angMomentum, 0, new Derivative());
        b = evaluate(position, linearMomentum, angMomentum, dt * 0.5f, a);
        c = evaluate(position, linearMomentum, angMomentum, dt * 0.5f, b);
        d = evaluate(position, linearMomentum, angMomentum, dt, c);

        position = position + ((1.0f / 6.0f) * (a.dx + 2.0f * (b.dx + c.dx) + d.dx)) * dt;
        rotation = addQuaToQua(rotation, multiplyQuaByScalar((angVelocityQua * rotation), 0.5f * dt));

        // collision and stuff        
        collision(dt, otherParticles, aliveParticles);

        // Momentum
        linearMomentum = linearMomentum + ((1.0f / 6.0f) * (a.dm + 2.0f * (b.dm + c.dm) + d.dm)) * dt;
        //if (linearMomentum.magnitude < 0.001)
            //linearMomentum.Set(0, 0, 0);

        angMomentum = angMomentum + ((1.0f / 6.0f) * (a.dam + 2.0f * (b.dam + c.dam) + d.dam)) * dt;
        //if (angMomentum.magnitude < 0.001)
            //angMomentum.Set(0, 0, 0);        

        // Velocity
        linearVelocity = linearMomentum / mass;
        angVelocity = Vector3.Scale(new Vector3(1.0f / inertiaTensor.x, 1.0f / inertiaTensor.y, 1.0f / inertiaTensor.z), angMomentum);
        angVelocityQua = new Quaternion(angVelocity.x, angVelocity.y, angVelocity.z, 0);

        // Natural Roll ?
        naturalRoll = (Mathf.Abs(linearVelocity.magnitude - (radius * angVelocity.magnitude)) < 0.1);
    }


    Derivative evaluate(Vector3 initialPos, Vector3 initialMomentum, Vector3 initialAngMomentum, float dt, Derivative d)
    {
        Vector3 newPos = initialPos + d.dx * dt;
        Vector3 newMomentum = initialMomentum + d.dm * dt;
        Vector3 newAngMomentum = initialAngMomentum + d.dam * dt;

        Vector3 newVel = newMomentum / mass;
        Vector3 newAngVel = Vector3.Scale(new Vector3(1.0f / inertiaTensor.x, 1.0f / inertiaTensor.y, 1.0f / inertiaTensor.z), newAngMomentum);

        Derivative output = new Derivative();
        output.dx = newVel;
        output.dm = force(newVel);
        output.dam = torque(newVel);

        return output;
    }

    Vector3 force(Vector3 newVel)
    {
        Vector3 Fg = (9.8f * mass) * Vector3.down;
        Vector3 staticFriction = 0.8f * -(Fg + penaltyFt);
        Vector3 damper = 0.7f * -Vector3.Normalize(newVel);
        Vector3 result;

        if ((penaltyFl + Fg).magnitude < staticFriction.magnitude)
        {
            linearMomentum += -0.008f * linearMomentum;
        }
        
        result = Fl + damper + penaltyFl + Fg;
        return result;
    }

    Vector3 torque(Vector3 newVel)
    {
        if (!naturalRoll)
        {
            Vector3 damper = 0.5f * -Vector3.Normalize(newVel);
            Ft = Vector3.Cross(new Vector3(0, -0.5f, 0), damper);
        }
        else
        {
            Vector3 damper = 0.02f * -Vector3.Normalize(newVel);
            Ft = -Vector3.Cross(new Vector3(0, -0.5f, 0), damper);
        }

        // this is a force contrary to roll to guarantee the ball will stop rolling evenrually
        if (angMomentum.magnitude != 0) {
            angMomentum += -0.008f * angMomentum;
        }

        return Ft + penaltyFt;
    }
   

    void collision(float dt, SandParticle[] otherParticles, int aliveParticles) {
        Vector3 pos;
        float distance;

        float kn = 60;
        float ys = 3;
        float u = 0.05f;
        float ks = 20;

        penaltyFl.Set(0, 0, 0);
        penaltyFt.Set(0, 0, 0);

        //floorCollision = (transform.position.y <= 0.5);
        if (position.y <= radius)
        {
            distance = Vector3.Distance(new Vector3(position.x, 0, position.z), position);
            Vector3 N = Vector3.Normalize(new Vector3(position.x, 0, position.z) - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 0.5f - distance;
            float ep = Vector3.Dot(relativeVelocity, N);

            Vector3 Fn = -(40 * Mathf.Pow(e, 3 / 2) + 10 * ep * Mathf.Pow(e, 1 / 2)) * N;

            penaltyFl += Fn;
        }

        // walls
        pos = GameObject.Find("Wall1").transform.position;
        distance = Vector3.Distance(new Vector3(pos.x, position.y, position.z), position);
        if (distance <= 1)
        {
            Vector3 N = Vector3.Normalize(new Vector3(pos.x, position.y, position.z) - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 1 - distance;
            float ep = Vector3.Dot(relativeVelocity, N);

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 40 * ep * Mathf.Pow(e, 1 / 2)) * N;

            penaltyFl += Fn;
        }

        pos = GameObject.Find("Wall2").transform.position;
        distance = Vector3.Distance(new Vector3(pos.x, position.y, position.z), position);
        if (distance <= 1)
        {
            Vector3 N = Vector3.Normalize(new Vector3(pos.x, position.y, position.z) - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 1 - distance;
            float ep = Vector3.Dot(relativeVelocity, N);

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 40 * ep * Mathf.Pow(e, 1 / 2)) * N;

            penaltyFl += Fn;
        }

        pos = GameObject.Find("Wall3").transform.position;
        distance = Vector3.Distance(new Vector3(position.x, position.y, pos.z), position);
        if (distance <= 1)
        {
            Vector3 N = Vector3.Normalize(new Vector3(position.x, position.y, pos.z) - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 1 - distance;
            float ep = Vector3.Dot(relativeVelocity, N);

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 40 * ep * Mathf.Pow(e, 1 / 2)) * N;

            penaltyFl += Fn;
        }

        pos = GameObject.Find("Wall4").transform.position;
        distance = Vector3.Distance(new Vector3(position.x, position.y, pos.z), position);
        if (distance <= 1)
        {
            Vector3 N = Vector3.Normalize(new Vector3(position.x, position.y, pos.z) - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 1 - distance;
            float ep = Vector3.Dot(relativeVelocity, N);

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 40 * ep * Mathf.Pow(e, 1 / 2)) * N;

            penaltyFl += Fn;
        }

        //obj collisions
        pos = GameObject.Find("Obj1").transform.position;
        distance = Vector3.Distance(pos, position);
        if (distance <= 8) {
            Vector3 N = Vector3.Normalize(pos - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 8 - distance;
            float ep = Vector3.Dot(relativeVelocity, N);
            Vector3 tangentialVelocity = relativeVelocity - ep * N;

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 30 * ep * Mathf.Pow(e, 1 / 2)) * N;
            Fn += 0.7f * Vector3.Normalize(linearVelocity);//Add some value to affect the friction
            Vector3 Ft = -Mathf.Min(0.5f * Fn.magnitude, 100 * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

            penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
            penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
        }

        pos = GameObject.Find("Obj2").transform.position;
        distance = Vector3.Distance(pos, position);
        if (distance <= 5.5f)
        {
            Vector3 N = Vector3.Normalize(pos - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 5.5f - distance;
            float ep = Vector3.Dot(relativeVelocity, N);
            Vector3 tangentialVelocity = relativeVelocity - ep * N;

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 30 * ep * Mathf.Pow(e, 1 / 2)) * N;
            Fn += 0.7f * Vector3.Normalize(linearVelocity);//Add some value to affect the friction
            Vector3 Ft = -Mathf.Min(0.5f * Fn.magnitude, 100 * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

            penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
            penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
        }

        pos = GameObject.Find("Obj3").transform.position;
        distance = Vector3.Distance(pos, position);
        if (distance <= 4.5f)
        {
            Vector3 N = Vector3.Normalize(pos - position);
            Vector3 relativeVelocity = linearVelocity;

            float e = 4.5f - distance;
            float ep = Vector3.Dot(relativeVelocity, N);
            Vector3 tangentialVelocity = relativeVelocity - ep * N;

            Vector3 Fn = -(50 * Mathf.Pow(e, 3 / 2) + 30 * ep * Mathf.Pow(e, 1 / 2)) * N;
            Fn += 0.7f * Vector3.Normalize(linearVelocity);//Add some value to affect the friction
            Vector3 Ft = -Mathf.Min(0.5f * Fn.magnitude, 100 * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

            penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
            penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
        }

        // Sphere
        pos = GameObject.Find("Sphere").transform.position;
        distance = Vector3.Distance(pos, position);

        if (distance <= radius + 5 )
        {
            Vector3 N = Vector3.Normalize(pos - position);
            Vector3 relativeVelocity = linearVelocity - GameObject.Find("Sphere").GetComponent<MetalSphere>().linearVelocity;

            float e = (radius + 5) - distance;
            float ep = Vector3.Dot(relativeVelocity, N);
            Vector3 tangentialVelocity = relativeVelocity - ep * N;

            Vector3 Fn = -(60 * Mathf.Pow(e, 3 / 2) + 60 * ep * Mathf.Pow(e, 1 / 2)) * N;
            Vector3 Ft = -Mathf.Min(u * Fn.magnitude, ks * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

            penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
            penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
        }

        // sphere objs
        for (int i = 1; i < 81; ++i)
        {
            pos = GameObject.Find("Sphere (" + i + ")").transform.position;
            distance = Vector3.Distance(pos, position);

            if (distance <= 2 * radius && distance != 0)
            {
                Vector3 N = Vector3.Normalize(pos - position);
                Vector3 relativeVelocity = linearVelocity;

                float e = 1 - distance;
                float ep = Vector3.Dot(relativeVelocity, N);
                Vector3 tangentialVelocity = relativeVelocity - ep * N;

                Vector3 Fn = -(60 * Mathf.Pow(e, 3 / 2) + 40 * ep * Mathf.Pow(e, 1 / 2)) * N;
                Vector3 Ft = -Mathf.Min(u * Fn.magnitude, ks * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

                penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
                penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
            }
        }

        for (int i = 0; i < aliveParticles; ++i)
        {
            pos = otherParticles[i].position;
            distance = Vector3.Distance(pos, position);

            if (distance <= 2*radius && distance != 0)
            {
                Vector3 N = Vector3.Normalize(pos - position);
                Vector3 relativeVelocity = linearVelocity - otherParticles[i].linearVelocity;

                float e = 1 - distance;
                float ep = Vector3.Dot(relativeVelocity, N);
                Vector3 tangentialVelocity = relativeVelocity - ep * N;

                Vector3 Fn = -(kn * Mathf.Pow(e, 3 / 2) + ys * ep * Mathf.Pow(e, 1 / 2)) * N;
                Vector3 Ft = -Mathf.Min(u * Fn.magnitude, ks * tangentialVelocity.magnitude) * Vector3.Cross(N / 2, tangentialVelocity).normalized;

                penaltyFl += Fn; //-(kn * Mathf.Pow(e, 3 / 2) - ys * penetrationRate1 * Mathf.Pow(e, 1 / 2)) * N;
                penaltyFt += Ft; // 0.3f * Vector3.Cross(new Vector3(0, -0.5f, 0), N / 2);
            }
        }
    }



    Quaternion multiplyQuaByScalar(Quaternion q, float s)
    {
        return new Quaternion(q.x * s, q.y * s, q.z * s, q.w * s);
    }

    Quaternion addQuaToQua(Quaternion q1, Quaternion q2)
    {
        return new Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
    }

    // Returns a normalized quaternion
    private Quaternion normalize(Quaternion q)
    {
        float den = Mathf.Sqrt(Mathf.Pow(q.x, 2) + Mathf.Pow(q.y, 2) + Mathf.Pow(q.z, 2) + Mathf.Pow(q.w, 2));
        return new Quaternion(q.x / den, q.y / den, q.z / den, q.w / den);
    }

}
