using Boids.Components;
using Latios.Psyshock;
using Latios.Transforms;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Collider = Latios.Psyshock.Collider;
using Physics = Latios.Psyshock.Physics;
using SphereCollider = Latios.Psyshock.SphereCollider;

namespace Boids.Systems.Boids
{
    [BurstCompile]
    public partial struct CollisionAvoidanceJob : IJobEntity
    {
        [ReadOnly] public CollisionLayer CollisionLayer;

        void Execute(TransformAspect transformAspect,
            BoidAspect boidAspect)
        {
            var position = transformAspect.worldPosition;
            var rayDirection = boidAspect.Velocity;

            if (math.length(rayDirection) < math.EPSILON)
                return;

            Collider collider = new SphereCollider
            {
                center = float3.zero,
                radius = .25f
            };

            var transform = new TransformQvvs
            {
                position = position + math.up(),
                rotation = quaternion.identity
            };

            if (Physics.ColliderCast(
                    in collider, in transform,
                    transform.position + rayDirection,
                    in CollisionLayer, out var result, out var info))
            {
                var avoidanceForce = result.normalOnTarget;
                avoidanceForce *= 1f / result.distance;
                boidAspect.CollisionAvoidanceForce =
                    math.any(math.isnan(avoidanceForce)) ? float3.zero : avoidanceForce;
            }
            else
            {
                boidAspect.CollisionAvoidanceForce = float3.zero;
            }
        }
    }
}