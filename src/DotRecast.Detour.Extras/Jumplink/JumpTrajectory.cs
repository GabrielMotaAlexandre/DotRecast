using System;
using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class JumpTrajectory : Trajectory
    {
        private readonly float jumpHeight;

        public JumpTrajectory(float jumpHeight)
        {
            this.jumpHeight = jumpHeight;
        }

        public override Vector3 Apply(Vector3 start, Vector3 end, float u)
        {
            return new Vector3
            {
                X = Lerp(start.X, end.X, u), 
                Y = InterpolateHeight(start.Y, end.Y, u),
                Z = Lerp(start.Z, end.Z, u)
            };
        }

        private float InterpolateHeight(float ys, float ye, float u)
        {
            if (u is 0f)
            {
                return ys;
            }
            else if (u == 1f)
            {
                return ye;
            }

            float h1, h2;
            if (ys >= ye)
            {
                // jump down
                h1 = jumpHeight;
                h2 = jumpHeight + ys - ye;
            }
            else
            {
                // jump up
                h1 = jumpHeight + ys - ye;
                h2 = jumpHeight;
            }

            float t = (float)(Math.Sqrt(h1) / (Math.Sqrt(h2) + Math.Sqrt(h1)));
            if (u <= t)
            {
                float v1 = 1f - (u / t);
                return ys + h1 - h1 * v1 * v1;
            }

            float v = (u - t) / (1f - t);
            return ys + h1 - h2 * v * v;
        }
    }
}