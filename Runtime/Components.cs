using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace Xacce.Susanin.Runtime
{
    public partial struct SusaninActor : IComponentData
    {
        public NavMeshLocation currentLocation;
    }

    public partial struct NavMeshSearchPath : IBufferElementData
    {
        public float3 from;
        public float3 to;
        public Entity response;
        public float3 extents;
        public int agentTypeId;
        public int areaMask;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create(in Entity entity, in float3 from, in float3 to, in float3 extents, int agentTypeId, int areaMask, out NavMeshSearchPath path)
        {
            path = new NavMeshSearchPath()
            {
                response = entity,
                from = from,
                to = to,
                areaMask = areaMask,
                extents = extents,
                agentTypeId = agentTypeId,
            };
        }
    }

    [InternalBufferCapacity(0)]
    public partial struct NavMeshPathElement : IBufferElementData
    {
        public PolygonId polygonId;
    }
}