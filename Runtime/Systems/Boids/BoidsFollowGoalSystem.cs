using Boids.Components;
using Latios;
using Latios.Transforms;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Boids.Systems.Boids
{
    public partial struct BoidsFollowGoalSystem : ISystem
    {
        EntityQuery m_query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_query = state.Fluent()
                .WithAspect<TransformAspect>()
                .WithAspect<BoidAspect>()
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

                if (math.any(math.isnan(goal))) return;

                var seek = boidAspect.Seek(transformAspect.worldPosition, goal);

                if (math.any(math.isnan(seek))) return;

                var distanceToGoal = math.distance(transformAspect.worldPosition, goal);
                if (distanceToGoal < boidAspect.Settings.agentRadius)
                    seek = math.lerp(float3.zero, seek, distanceToGoal / boidAspect.Settings.agentRadius);

                // var directionToGoal = math.normalize(goal - transformAspect.worldPosition);
                boidAspect.ApplyForce(seek * boidAspect.Settings.followStrength);
                //directionToGoal * boidAspect.Settings.followStrength);
            }
        }
    }
}