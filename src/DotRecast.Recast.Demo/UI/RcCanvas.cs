/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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


using Serilog;
using Silk.NET.Maths;
using Silk.NET.Windowing;

namespace DotRecast.Recast.Demo.UI
{
    public class RcCanvas
    {
        private readonly IWindow _window;
        private readonly IRcView[] _views;
        private bool _mouseOver;

        public bool IsMouseOver() => _mouseOver;

        public Vector2D<int> Size => _window.Size;

        public RcCanvas(IWindow window, params IRcView[] views)
        {
            _window = window;
            _views = views;
            foreach (var view in _views)
            {
                view.Bind(this);
            }
        }

        public void Update(double dt)
        {
            foreach (var view in _views)
            {
                view.Update(dt);
            }
        }

        public void Draw(double dt)
        {
            _mouseOver = false;
            foreach (var view in _views)
            {
                view.Draw(dt);
                _mouseOver |= view.IsHovered();
            }
        }
    }
}