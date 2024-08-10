using System.Runtime.CompilerServices;
using UnityEngine.Experimental.AI;

namespace Xacce.Susanin.Runtime
{
    public partial struct NavMeshQueryData
    {
        public enum _Status : byte
        {
            Fresh = 0,
            Idle = 1,
            Wait = 2,
        }

        public _Status _status;
        public PathQueryStatus status;
        public NavMeshSearchPath request;
        public int iterations;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Reset(ref NavMeshQueryData data, in NavMeshSearchPath request)
        {
            data.iterations = 0;
            data._status = _Status.Fresh;
            data.request = request;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Reset(ref NavMeshQueryData data)
        {
            data.iterations = 0;
            data._status = _Status.Idle;
        }

    }


}