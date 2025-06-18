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

        public float3 GoalPosition => m_goal.ValueRO.GoalPosition;
        public float3 Velocity => m_forces.ValueRO.Velocity;
        public BoidSettings Settings => m_settings.ValueRO;


        public void ClearNeighbors(int idx, Entity ent, NativeArray<ColliderBody> bodies, TransformQvvs transform)
        {
            Neighbors.Clear();

            var sphereCollider = new SphereCollider(float3.zero, m_settings.ValueRO.centeringRadius);

            bodies[idx] = new ColliderBody
            {
                collider  = sphereCollider,
                entity    = ent,
                transform = transform
            };
        }


        public float3 CalculateForces()
        {
            var alignment = m_forces.ValueRO.AlignmentForce;
            var centering = m_forces.ValueRO.CenteringForce;
            var avoidance = m_forces.ValueRO.AvoidanceForce;
            var follow = m_forces.ValueRO.FollowForce;

            var totalForce = alignment + centering + avoidance + follow;
            totalForce = math.lengthsq(totalForce) > float.Epsilon ? math.normalize(totalForce) : float3.zero;

            return totalForce;
        }


        public void DebugForces(float3 position)
        {
            var alignment = m_forces.ValueRO.AlignmentForce;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + alignment * 10f,
                Color.green);

            var centering = m_forces.ValueRO.CenteringForce;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + centering * 10f,
                Color.blue);

            var avoidance = m_forces.ValueRO.AvoidanceForce;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + avoidance * 10f,
                Color.red);

            var follow = m_forces.ValueRO.FollowForce;
            Debug.DrawLine(
                position + math.up(),
                position + math.up() + follow * 10f,
                Color.yellow, 1f, false);
        }


        public float3 Seek(float3 position, float3 target)
        {
            var desired = math.normalize(target - position) * m_settings.ValueRO.maxSpeed;
            var steering = desired - m_forces.ValueRO.Velocity;

            if (math.length(steering) > m_settings.ValueRO.maxSpeed)
                steering = math.normalize(steering) * m_settings.ValueRO.maxSpeed;

            if (math.any(math.isnan(steering))) return float3.zero;

            return steering;
        }

        public void ApplyForce(float3 force)
        {
            // check for nan force
            if (math.any(math.isnan(force)))
            {
                Debug.LogWarning($"Force applied to boid is NaN: {force}");
                return;
            }

            m_forces.ValueRW.Acceleration += force;
        }

        public float3 Update(float3 position, float deltaTime)
        {
            // Update velocity
            m_forces.ValueRW.Velocity += m_forces.ValueRO.Acceleration * deltaTime;

            if (math.length(m_forces.ValueRO.Velocity) > m_settings.ValueRO.maxSpeed)
                m_forces.ValueRW.Velocity = math.normalize(m_forces.ValueRO.Velocity) * m_settings.ValueRO.maxSpeed;

            // Update position
            var newPosition = position + m_forces.ValueRW.Velocity * deltaTime;
            // Reset acceleration for next frame
            m_forces.ValueRW.Acceleration = float3.zero;

            return newPosition;
        }

        public void SetGoal(float3 goal)
        {
            m_goal.ValueRW.GoalPosition = goal;
        }
    }
}