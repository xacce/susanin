#if UNITY_EDITOR
using Unity.Entities;
using UnityEngine;
using Xacce.Susanin.Runtime;

namespace Xacce.Susanin.Hybrid
{
    public class SusaninActorAuthoring : MonoBehaviour
    {
        private class SusaninAactorBaker : Baker<SusaninActorAuthoring>
        {
            public override void Bake(SusaninActorAuthoring authoring)
            {
                var e = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<SusaninActor>(e);
                AddBuffer<NavMeshPathElement>(e);
            }
        }
    }
}
#endif