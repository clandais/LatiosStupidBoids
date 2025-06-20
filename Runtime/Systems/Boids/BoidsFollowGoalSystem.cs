using Boids.Authoring;
using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    internal partial struct BoidsFollowGoalSystem : ISystem
    {
        EntityQuery m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_query = state.Fluent()
                .WithAspect<TransformAspect>()
                .WithAspect<BoidAspect>()
                .WithEnabled<BoidEnabledTag>()
                .Build();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Dependency = new FollowGoalJob().ScheduleParallel(m_query, state.Dependency);
        }

        [BurstCompile]
        partial struct FollowGoalJob : IJobEntity
        {
            void Execute(
                TransformAspect transformAspect,
                BoidAspect boidAspect)
            {
                var goal = boidAspect.GoalPosition;
                var force = boidAspect.Arrive(transformAspect.worldPosition, goal);
                if (math.any(math.isnan(force)))
                {
                    boidAspect.FollowForce = float3.zero;
                    return;
                }

                boidAspect.FollowForce = force;
            }
        }
    }
}