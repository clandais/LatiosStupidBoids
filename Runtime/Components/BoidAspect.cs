using Boids.Authoring;
using Latios.Psyshock;
using Latios.Transforms;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using SphereCollider = Latios.Psyshock.SphereCollider;

namespace Boids.Components
{
    public readonly partial struct BoidAspect : IAspect
    {
        readonly RefRO<BoidSettings>         m_settings;
        readonly DynamicBuffer<BoidNeighbor> m_neighbors;
        readonly RefRW<BoidForces>           m_forces;
        readonly RefRW<BoidGoal>             m_goal;

        public int NeighborCount => Neighbors.Length;

        // ReSharper disable once ConvertToAutoProperty
        public DynamicBuffer<BoidNeighbor> Neighbors => m_neighbors;

        public float3 SeparationForce
        {
            get => m_forces.ValueRO.Separation;
            set => m_forces.ValueRW.Separation = value;
        }

        public float3 AlignmentForce
        {
            get => m_forces.ValueRO.Alignment;
            set => m_forces.ValueRW.Alignment = value;
        }

        public float3 CohesionForce
        {
            get => m_forces.ValueRO.Cohesion;
            set => m_forces.ValueRW.Cohesion = value;
        }

        public float3 FollowForce
        {
            get => m_forces.ValueRO.Follow;
            set => m_forces.ValueRW.Follow = value;
        }

        public float3 CollisionAvoidanceForce
        {
            get => m_forces.ValueRO.CollisionAvoidance;
            set => m_forces.ValueRW.CollisionAvoidance = value;
        }


        public float3 GoalPosition => m_goal.ValueRO.GoalPosition;


        public float3 Velocity
        {
            get => m_forces.ValueRO.Velocity;
            set => m_forces.ValueRW.Velocity = value;
        }

        public BoidSettings Settings => m_settings.ValueRO;


        public void ClearNeighbors(int idx, Entity ent, NativeArray<ColliderBody> bodies, TransformQvvs transform)
        {
            Neighbors.Clear();

            var sphereCollider = new SphereCollider(float3.zero, m_settings.ValueRO.cohesionRadius);

            bodies[idx] = new ColliderBody
            {
                collider  = sphereCollider,
                entity    = ent,
                transform = transform
            };
        }


        public float3 Arrive(float3 position, float3 target)
        {
            var toTarget = target - position;
            var distance = math.distance(position, target);
            var desired = float3.zero;
            var slowdownRadius = m_settings.ValueRO.slowingRadius;

            if (distance <= math.EPSILON)
                return desired;

            var ramped = m_settings.ValueRO.maxSpeed * (distance / (slowdownRadius * m_settings.ValueRO.agentWeight));
            var clamped = math.min(ramped, m_settings.ValueRO.maxSpeed);
            desired = clamped * (toTarget / distance);

            desired -= m_forces.ValueRO.Velocity;
            return desired;
        }

        float3 CalculateForces()
        {
            var alignment = m_forces.ValueRO.Alignment * m_settings.ValueRO.alignmentWeight;
            var centering = m_forces.ValueRO.Cohesion * m_settings.ValueRO.cohesionWeight;
            var avoidance = m_forces.ValueRO.Separation * m_settings.ValueRO.separationWeight;
            var follow = m_forces.ValueRO.Follow * m_settings.ValueRO.followStrength;
            var collisionAvoidance = m_forces.ValueRO.CollisionAvoidance * m_settings.ValueRO.collisionAvoidanceWeight;

            var totalForce = alignment + centering + avoidance + follow + collisionAvoidance;
            totalForce = math.length(totalForce) > m_settings.ValueRO.maxForce
                ? math.normalize(totalForce) * m_settings.ValueRO.maxForce
                : totalForce;

            return totalForce;
        }


        public void DebugForces(float3 position)
        {
            var alignment = m_forces.ValueRO.Alignment;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + alignment * 5f,
                Color.blue);

            var centering = m_forces.ValueRO.Cohesion;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + centering * 5f,
                Color.yellow);

            var avoidance = m_forces.ValueRO.Separation;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + avoidance * 5f,
                Color.magenta);

            var follow = m_forces.ValueRO.Follow;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + follow * 5f,
                Color.cyan);

            var collisionAvoidance = m_forces.ValueRO.CollisionAvoidance;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + collisionAvoidance * 5f,
                Color.red);

            var totalForce = CalculateForces();
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + totalForce * 5f,
                Color.green);
        }


        public float3 GetSteering(float deltaTime)
        {
            var force = CalculateForces();
            if (math.any(math.isnan(force))) return float3.zero;
            return force * deltaTime;
        }

        public void SetGoal(float3 goal)
        {
            m_goal.ValueRW.GoalPosition = goal;
        }
    }
}