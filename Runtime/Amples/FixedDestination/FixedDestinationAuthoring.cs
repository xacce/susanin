#if UNITY_EDITOR
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Amples.FixedDestination
{
    public class FixedDestinationAuthoring : MonoBehaviour
    {
        [SerializeField] private float3 point;
        [SerializeField] private Transform target;

        private class DemoAgentRandomDestinationsBaker : Baker<FixedDestinationAuthoring>
        {
            public override void Bake(FixedDestinationAuthoring authoring)
            {
                var e = GetEntity(authoring, TransformUsageFlags.None);
                AddComponent(
                    e,
                    new DemoAgentFixedDestination()
                    {
                        point = authoring.target == null ? authoring.point : authoring.target.position,
                    });

            }
        }
    }
}
#endif