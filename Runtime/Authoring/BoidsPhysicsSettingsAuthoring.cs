using Latios.Psyshock;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Boids.Authoring
{
    public class BoidsPhysicsSettingsAuthoring : MonoBehaviour
    {
        [SerializeField] Aabb worldAabb;
        [SerializeField] int3 subdivisions = new(1, 1, 8);

        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.yellow;
            var center = (worldAabb.min + worldAabb.max) * 0.5f;
            var size = worldAabb.max - worldAabb.min;
            Gizmos.DrawWireCube(center, size);
        }

        class BoidsPhysicsSettingsAuthoringBaker : Baker<BoidsPhysicsSettingsAuthoring>
        {
            public override void Bake(BoidsPhysicsSettingsAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new BoidsPhysicsSettings
                {
                    collisionLayerSettings = new CollisionLayerSettings
                    {
                        worldAabb                = authoring.worldAabb,
                        worldSubdivisionsPerAxis = authoring.subdivisions
                    }
                });
            }
        }
    }


    public struct BoidsPhysicsSettings : IComponentData
    {
        public CollisionLayerSettings collisionLayerSettings;
    }
}