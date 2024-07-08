#if UNITY_EDITOR
using Unity.Entities;
using UnityEngine;

namespace Susanin.Hybrid
{
    public class SusaninActor : MonoBehaviour
    {
        private class SusaninAactorBaker : Baker<SusaninActor>
        {
            public override void Bake(SusaninActor authoring)
            {
                var e = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<AgentFollowToDirectionTag>(e);
                AddComponent<global::SusaninActor>(e);
                AddBuffer<NavMeshPathElement>(e);
            }
        }
    }
}
#endif