/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org
DotRecast Copyright (c) 2023 Choi Ikpil ikpil@naver.com

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Numerics;
using NUnit.Framework;

namespace DotRecast.Detour.Test
{
    [Parallelizable]
    public class PolygonByCircleConstraintTest
    {
        private readonly DtStrictDtPolygonByCircleConstraint _constraint = DtStrictDtPolygonByCircleConstraint.Shared;

        [Test]
        public void ShouldHandlePolygonFullyInsideCircle()
        {
            var polygon = new float[] { -2, 0, 2, 2, 0, 2, 2, 0, -2, -2, 0, -2 }.AsSpan().Cast<float, Vector3>().ToArray();
            Vector3 center = new(1, 0, 1);
            var constrained = _constraint.Apply(polygon, center, 6).ToArray();

            Assert.That(constrained, Is.EqualTo(polygon));
        }

        [Test]
        public void ShouldHandleVerticalSegment()
        {
            int expectedSize = 7;
            var polygon = new float[] { -2, 0, 2, 2, 0, 2, 2, 0, -2, -2, 0, -2 }.AsSpan().Cast<float, Vector3>();
            Vector3 center = new(2, 0, 0);

            var constrained = _constraint.Apply(polygon, center, 3).ToArray();
            Assert.That(constrained, Has.Length.EqualTo(expectedSize));
            Assert.That(constrained, Is.SupersetOf(new[] { 2f, 0f, 2f, 2f, 0f, -2f }.AsSpan().Cast<float, Vector3>().ToArray()));
        }

        [Test]
        public void ShouldHandleCircleFullyInsidePolygon()
        {
            int expectedSize = 12;
            var polygon = new float[] { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 }.AsSpan().Cast<float, Vector3>();
            Vector3 center = new(-1, 0, -1);
            var constrained = _constraint.Apply(polygon, center, 2).ToArray();

            Assert.That(constrained, Has.Length.EqualTo(expectedSize));

            for (int i = 0; i < expectedSize; i++)
            {
                var v2 = constrained[i].AsVector2XZ() + Vector2.One;
                Assert.That(v2.LengthSquared(), Is.EqualTo(4).Within(1e-4f));
            }
        }

        [Test]
        public void ShouldHandleCircleInsidePolygon()
        {
            int expectedSize = 9;
            var polygon = new float[] { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 }.AsSpan().Cast<float, Vector3>();
            Vector3 center = new(-2, 0, -1);
            var constrained = _constraint.Apply(polygon, center, 3).ToArray();

            Assert.That(constrained, Has.Length.EqualTo(expectedSize));
            Assert.That(constrained, Is.SupersetOf(new[] { -2f, 0f, -4f, -4f, 0f, 0f, -3.4641016f, 0f, 1.6076951f, -2f, 0f, 2f }.AsSpan().Cast<float, Vector3>().ToArray()));
        }

        [Test]
        public void ShouldHandleCircleOutsidePolygon()
        {
            int expectedSize = 7;
            var polygon = new float[] { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 }.AsSpan().Cast<float, Vector3>();
            Vector3 center = new(4, 0, 0);
            var constrained = _constraint.Apply(polygon, center, 4).ToArray();

            Assert.That(constrained, Has.Length.EqualTo(expectedSize));
            Assert.That(constrained, Is.SupersetOf(new[] { 1.5358982f, 0f, 3f, 2f, 0f, 3f, 3f, 0f, -3f }.AsSpan().Cast<float, Vector3>().ToArray()));
        }
    }
}