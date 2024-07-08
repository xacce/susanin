using Unity.Entities;
using UnityEngine;

#if UNITY_EDITOR
namespace Amples.RandomAgents
{
    public class DemoAgentRandomDestinationsAuthoring : MonoBehaviour
    {
        [SerializeField] private float delay;
        [SerializeField] private float radius;

        private class DemoAgentRandomDestinationsBaker : Baker<DemoAgentRandomDestinationsAuthoring>
        {
            public override void Bake(DemoAgentRandomDestinationsAuthoring authoring)
            {
                var e = GetEntity(authoring, TransformUsageFlags.None);
                AddComponent(
                    e,
                    new DemoAgentRandomDestinations()
                    {
                        currentDelay = 0,
                        delay = authoring.delay,
                        radius = authoring.radius,
                    });

            }
        }
    }
}
#endif