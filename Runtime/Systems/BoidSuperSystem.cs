using Boids.Systems.Boids;
using Latios;
using Latios.Transforms.Systems;
using Unity.Entities;

namespace Boids.Systems
{
    [UpdateInGroup(typeof(PreTransformSuperSystem))]
    public partial class BoidSuperSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            GetOrCreateAndAddUnmanagedSystem<BoidsInitializationSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsCenterSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsAvoidanceSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsAlignmentSystem>();
            GetOrCreateAndAddUnmanagedSystem<BoidsFollowGoalSystem>();
        }
    }
}